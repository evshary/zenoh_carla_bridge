use atomic_float::AtomicF32;
use log::info;
use std::sync::{
    atomic::Ordering,
    mpsc::{self, Receiver},
    Arc,
};

use cdr::{CdrLe, Infinite};
use zenoh::{
    buffers::reader::HasReader, prelude::sync::*, publication::Publisher, subscriber::Subscriber,
};

use carla::{
    client::{ActorBase, Vehicle},
    rpc::{VehicleControl, VehicleWheelLocation},
};

use carla_ackermann::{
    vehicle_control::{Output, TargetRequest},
    VehicleController,
};

use crate::autoware_type::{
    self, AckermannControlCommand, AckermannLateralCommand, LongitudinalCommand,
};

pub struct VehicleBridge<'a> {
    _vehicle_name: String,
    actor: Vehicle,
    _subscriber_control_cmd: Subscriber<'a, ()>,
    _subscriber_gear_cmd: Subscriber<'a, ()>,
    publisher_velocity: Publisher<'a>,
    speed: Arc<AtomicF32>,
    controller: VehicleController,
    cmd_rx: Receiver<AckermannControlCommand>,
}

impl<'a> VehicleBridge<'a> {
    pub fn new(z_session: &'a Session, name: String, actor: Vehicle) -> VehicleBridge<'a> {
        let (cmd_tx, cmd_rx) = mpsc::sync_channel(10);

        let physics_control = actor.physics_control();
        let controller = VehicleController::from_physics_control(&physics_control, None);

        let publisher_velocity = z_session
            // TODO: Check whether Zenoh can receive the message
            .declare_publisher(name.clone() + "/rt/vehicle/status/velocity_status")
            .res()
            .unwrap();
        let speed = Arc::new(AtomicF32::new(0.0));

        let subscriber_control_cmd = z_session
            .declare_subscriber(name.clone() + "/rt/external/selected/control_cmd")
            .callback_mut(move |sample| {
                let result: Result<autoware_type::AckermannControlCommand, _> =
                    cdr::deserialize_from(sample.payload.reader(), cdr::size::Infinite);
                let Ok(cmd) = result else {
                    return;
                };
                cmd_tx.send(cmd).unwrap();

                //let mut control = vehicle_actor.control();
                //// The algorithm is from https://github.com/hatem-darweesh/op_bridge/blob/ros2/op_bridge/op_ros2_agent.py#L219
                //// TODO: Check whether it works while reverse.
                //let speed_diff = cmd.longitudinal.speed - current_speed.load(Ordering::Relaxed);
                //if speed_diff > 0.0 {
                //    control.throttle = 0.75;
                //    control.brake = 0.0;
                //} else if speed_diff < 0.0 {
                //    control.throttle = 0.0;
                //    control.brake = if cmd.longitudinal.speed <= 0.0 {
                //        0.75
                //    } else {
                //        0.01
                //    };
                //}
                //info!(
                //    "target:{} current:{} diff:{}",
                //    cmd.longitudinal.speed,
                //    cmd.longitudinal.speed - speed_diff,
                //    speed_diff
                //);
                //// Transform the angle from radian to degree and calculate the ratio based on max wheel angle
                //control.steer =
                //    -cmd.lateral.steering_tire_angle * 180.0 / 3.14 / max_wheel_steer_angle;
                //vehicle_actor.apply_control(&control);
                //info!(
                //    "throttle: {}, break: {}, steer: {}\r",
                //    control.throttle,
                //    control.brake,
                //    -cmd.lateral.steering_tire_angle * 180.0 / 3.14
                //);
            })
            .res()
            .unwrap();
        let _subscriber_gate_mode = z_session
            .declare_subscriber(name.clone() + "/rt/control/gate_mode_cmd")
            .callback_mut(move |_| {
                // TODO
            })
            .res()
            .unwrap();
        //let mut vehicle_actor = actor.clone();
        let subscriber_gear_cmd = z_session
            .declare_subscriber(name.clone() + "/rt/external/selected/gear_cmd")
            .callback_mut(move |_sample| {
                // TODO
                //match cdr::deserialize_from::<_, autoware_type::GearCommand, _>(
                //    sample.payload.reader(),
                //    cdr::size::Infinite,
                //) {
                //    Ok(gearcmd) => {
                //        let mut control = vehicle_actor.control();
                //        control.reverse = gearcmd.command == autoware_type::GEAR_CMD_REVERSE;
                //        control.gear = if gearcmd.command == autoware_type::GEAR_CMD_REVERSE {
                //            -1
                //        } else {
                //            1
                //        };
                //        vehicle_actor.apply_control(&control);
                //    }
                //    Err(_) => {}
                //}
            })
            .res()
            .unwrap();

        VehicleBridge {
            _vehicle_name: name,
            actor,
            _subscriber_control_cmd: subscriber_control_cmd,
            _subscriber_gear_cmd: subscriber_gear_cmd,
            publisher_velocity,
            speed,
            controller,
            cmd_rx,
        }
    }

    fn pub_current_velocity(&mut self) {
        //let transform = vehicle_actor.transform();
        let velocity = self.actor.velocity();
        //let angular_velocity = vehicle_actor.angular_velocity();
        //let accel = vehicle_actor.acceleration();
        let velocity_msg = autoware_type::CurrentVelocity {
            header: autoware_type::StdMsgsHeader {
                // TODO: Use correct timestamp
                ts: autoware_type::TimeStamp { sec: 0, nsec: 0 },
                frameid: String::from(""),
            },
            longitudinal_velocity: velocity.norm(),
            lateral_velocity: 0.0,
            // The heading rate is 1 deg to 0.00866, and the direction is reverse
            heading_rate: self
                .actor
                .get_wheel_steer_angle(VehicleWheelLocation::FL_Wheel)
                * -0.00866,
        };
        info!(
            "Carla => Autoware: current velocity: {}",
            velocity_msg.longitudinal_velocity
        );
        let encoded = cdr::serialize::<_, _, CdrLe>(&velocity_msg, Infinite).unwrap();
        self.publisher_velocity.put(encoded).res().unwrap();
        self.speed
            .store(velocity_msg.longitudinal_velocity, Ordering::Relaxed);
        //info!("{}", velocity_msg.longitudinal_velocity);
    }

    fn update_carla_control(&mut self, elapsed_sec: f64) {
        if let Ok(cmd) = self.cmd_rx.try_recv() {
            let AckermannControlCommand {
                lateral:
                    AckermannLateralCommand {
                        steering_tire_angle,
                        ..
                    },
                longitudinal:
                    LongitudinalCommand {
                        speed,
                        acceleration,
                        ..
                    },
                ..
            } = cmd;
            info!(
                "Autoware => Carla: speed:{} accel:{} steering_tire_angle:{}",
                speed,
                acceleration,
                -steering_tire_angle.to_degrees()
            );
            let current_speed = self.actor.velocity().norm();
            let (_, pitch_radians, _) = self.actor.transform().rotation.euler_angles();
            self.controller.set_target(TargetRequest {
                steering_angle: -steering_tire_angle.to_degrees() as f64,
                speed: speed as f64,
                accel: acceleration as f64,
            });
            info!(
                "Autoware => Carla: elapse_sec:{} current_speed:{} pitch_radians:{}",
                elapsed_sec, current_speed, pitch_radians
            );
            let (
                Output {
                    throttle,
                    brake,
                    steer,
                    reverse,
                    hand_brake,
                },
                _,
            ) = self
                .controller
                .step(elapsed_sec, current_speed as f64, pitch_radians as f64);
            info!(
                "Autoware => Carla: throttle:{}, brake:{}, steer:{}",
                throttle, brake, steer
            );

            self.actor.apply_control(&VehicleControl {
                throttle: throttle as f32,
                steer: steer as f32,
                brake: brake as f32,
                hand_brake,
                reverse,
                manual_gear_shift: false,
                gear: 0,
            });
        }
    }

    pub fn step(&mut self, elapsed_sec: f64) {
        self.pub_current_velocity();
        self.update_carla_control(elapsed_sec);
    }
}
