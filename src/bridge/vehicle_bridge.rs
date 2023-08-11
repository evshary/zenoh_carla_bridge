use super::actor_bridge::ActorBridge;
use crate::{
    error::{Error, Result},
    utils,
};
use arc_swap::ArcSwap;
use atomic_float::AtomicF32;
use carla::{
    client::{ActorBase, Vehicle},
    rpc::{VehicleControl, VehicleWheelLocation},
};
use carla_ackermann::{
    vehicle_control::{Output, TargetRequest},
    VehicleController,
};
use cdr::{CdrLe, Infinite};
use log::{debug, info};
use std::sync::{atomic::Ordering, Arc};
use zenoh::{prelude::sync::*, publication::Publisher, subscriber::Subscriber};
use zenoh_ros_type::{
    autoware_auto_control_msgs::{
        AckermannControlCommand, AckermannLateralCommand, LongitudinalCommand,
    },
    autoware_auto_vehicle_msgs::{ControlModeReport, GearReport, SteeringReport, VelocityReport},
    builtin_interfaces::Time,
};

pub struct VehicleBridge<'a> {
    vehicle_name: String,
    actor: Vehicle,
    _subscriber_control_cmd: Subscriber<'a, ()>,
    _subscriber_gear_cmd: Subscriber<'a, ()>,
    publisher_velocity: Publisher<'a>,
    publisher_steer: Publisher<'a>,
    publisher_gear: Publisher<'a>,
    publisher_control: Publisher<'a>,
    speed: Arc<AtomicF32>,
    controller: VehicleController,
    current_ackermann_cmd: Arc<ArcSwap<AckermannControlCommand>>,
}

impl<'a> VehicleBridge<'a> {
    pub fn new(z_session: Arc<Session>, actor: Vehicle) -> Result<VehicleBridge<'a>> {
        let mut vehicle_name = actor
            .attributes()
            .iter()
            .find(|attr| attr.id() == "role_name")
            .unwrap()
            .value_string();

        // Remove "autoware_" in role name
        if !vehicle_name.starts_with("autoware_") {
            return Err(Error::Npc {
                npc_role_name: vehicle_name,
            });
        } else {
            vehicle_name = vehicle_name.replace("autoware_", "");
        }

        info!("Detect a vehicle {vehicle_name}");
        let physics_control = actor.physics_control();
        let controller = VehicleController::from_physics_control(&physics_control, None);

        let publisher_velocity = z_session
            .declare_publisher(format!("{vehicle_name}/rt/vehicle/status/velocity_status"))
            .res()?;
        let publisher_steer = z_session
            .declare_publisher(format!("{vehicle_name}/rt/vehicle/status/steering_status"))
            .res()?;
        let publisher_gear = z_session
            .declare_publisher(format!("{vehicle_name}/rt/vehicle/status/gear_status"))
            .res()?;
        let publisher_control = z_session
            .declare_publisher(format!("{vehicle_name}/rt/vehicle/status/control_mode"))
            .res()?;
        let speed = Arc::new(AtomicF32::new(0.0));

        // TODO: We can use default value here
        let current_ackermann_cmd = Arc::new(ArcSwap::from_pointee(AckermannControlCommand {
            stamp: Time { sec: 0, nanosec: 0 },
            lateral: AckermannLateralCommand {
                stamp: Time { sec: 0, nanosec: 0 },
                steering_tire_angle: 0.0,
                steering_tire_rotation_rate: 0.0,
            },
            longitudinal: LongitudinalCommand {
                stamp: Time { sec: 0, nanosec: 0 },
                speed: 0.0,
                acceleration: 0.0,
                jerk: 0.0,
            },
        }));
        let cloned_cmd = current_ackermann_cmd.clone();
        let subscriber_control_cmd = z_session
            .declare_subscriber(format!("{vehicle_name}/rt/control/command/control_cmd"))
            .callback_mut(move |sample| {
                let result: Result<AckermannControlCommand, _> =
                    cdr::deserialize_from(&*sample.payload.contiguous(), cdr::size::Infinite);
                let Ok(cmd) = result else {
                    return;
                };
                cloned_cmd.store(Arc::new(cmd));
            })
            .res()?;
        let subscriber_gear_cmd = z_session
            .declare_subscriber(format!("{vehicle_name}/rt/control/command/gear_cmd"))
            .callback_mut(move |_sample| {
                // TODO: We don't this now, since reverse will be calculated while subscribing control_cmd
            })
            .res()?;

        Ok(VehicleBridge {
            vehicle_name,
            actor,
            _subscriber_control_cmd: subscriber_control_cmd,
            _subscriber_gear_cmd: subscriber_gear_cmd,
            publisher_velocity,
            publisher_steer,
            publisher_gear,
            publisher_control,
            speed,
            controller,
            current_ackermann_cmd,
        })
    }

    fn pub_current_velocity(&mut self, timestamp: f64) -> Result<()> {
        //let transform = vehicle_actor.transform();
        let velocity = self.actor.velocity();
        //let angular_velocity = vehicle_actor.angular_velocity();
        //let accel = vehicle_actor.acceleration();
        let mut header = utils::create_ros_header(Some(timestamp)).unwrap();
        header.frame_id = String::from("base_link");
        let velocity_msg = VelocityReport {
            header,
            longitudinal_velocity: velocity.norm(),
            lateral_velocity: 0.0,
            heading_rate: self
                .actor
                .get_wheel_steer_angle(VehicleWheelLocation::FL_Wheel)
                .to_radians()
                * -1.0,
        };
        debug!(
            "Carla => Autoware: current velocity: {}",
            velocity_msg.longitudinal_velocity
        );
        let encoded = cdr::serialize::<_, _, CdrLe>(&velocity_msg, Infinite)?;
        self.publisher_velocity.put(encoded).res()?;
        self.speed
            .store(velocity_msg.longitudinal_velocity, Ordering::Relaxed);

        Ok(())
    }

    fn pub_current_steer(&mut self, timestamp: f64) -> Result<()> {
        let steer_msg = SteeringReport {
            stamp: Time {
                sec: timestamp.floor() as i32,
                nanosec: (timestamp.fract() * 1000_000_000_f64) as u32,
            },
            steering_tire_angle: self
                .actor
                .get_wheel_steer_angle(VehicleWheelLocation::FL_Wheel)
                .to_radians()
                * -1.0,
        };
        let encoded = cdr::serialize::<_, _, CdrLe>(&steer_msg, Infinite)?;
        self.publisher_steer.put(encoded).res()?;
        Ok(())
    }

    fn pub_current_gear(&mut self, timestamp: f64) -> Result<()> {
        let gear_msg = GearReport {
            stamp: Time {
                sec: timestamp.floor() as i32,
                nanosec: (timestamp.fract() * 1000_000_000_f64) as u32,
            },
            report: if self.actor.control().reverse { 20 } else { 2 }, // TODO: Use enum (20: reverse, 2: drive)
        };
        let encoded = cdr::serialize::<_, _, CdrLe>(&gear_msg, Infinite)?;
        self.publisher_gear.put(encoded).res()?;
        Ok(())
    }

    fn pub_current_control(&mut self, timestamp: f64) -> Result<()> {
        let control_msg = ControlModeReport {
            stamp: Time {
                sec: timestamp.floor() as i32,
                nanosec: (timestamp.fract() * 1000_000_000_f64) as u32,
            },
            mode: 1, // 1: AUTONOMOUS, 4: MANUAL. TODO: Now we don't have any way to switch these two modes.
        };
        let encoded = cdr::serialize::<_, _, CdrLe>(&control_msg, Infinite)?;
        self.publisher_control.put(encoded).res()?;
        Ok(())
    }

    fn update_carla_control(&mut self, elapsed_sec: f64) {
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
        } = **self.current_ackermann_cmd.load();
        debug!(
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
        debug!(
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
        debug!(
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

    pub fn vehicle_name(&self) -> &str {
        &self.vehicle_name
    }
}

impl<'a> ActorBridge for VehicleBridge<'a> {
    fn step(&mut self, elapsed_sec: f64, timestamp: f64) -> Result<()> {
        self.pub_current_velocity(timestamp)?;
        self.pub_current_steer(timestamp)?;
        self.pub_current_gear(timestamp)?;
        self.pub_current_control(timestamp)?;
        self.update_carla_control(elapsed_sec);
        Ok(())
    }
}

impl<'a> Drop for VehicleBridge<'a> {
    fn drop(&mut self) {
        info!("Remove vehicle name {}", self.vehicle_name());
    }
}
