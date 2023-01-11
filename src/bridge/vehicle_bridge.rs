use super::actor_bridge::ActorBridge;
use crate::error::Result;
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
use r2r::{
    autoware_auto_control_msgs::msg::{
        AckermannControlCommand, AckermannLateralCommand, LongitudinalCommand,
    },
    autoware_auto_vehicle_msgs::msg::VelocityReport,
    builtin_interfaces::msg::Time,
    std_msgs::msg::Header,
};
use std::sync::{atomic::Ordering, Arc};
use zenoh::{
    buffers::reader::HasReader, prelude::sync::*, publication::Publisher, subscriber::Subscriber,
};

pub struct VehicleBridge<'a> {
    vehicle_name: String,
    actor: Vehicle,
    _subscriber_control_cmd: Subscriber<'a, ()>,
    _subscriber_gear_cmd: Subscriber<'a, ()>,
    publisher_velocity: Publisher<'a>,
    speed: Arc<AtomicF32>,
    controller: VehicleController,
    current_ackermann_cmd: Arc<ArcSwap<AckermannControlCommand>>,
}

impl<'a> VehicleBridge<'a> {
    pub fn new(z_session: &'a Session, actor: Vehicle) -> Result<VehicleBridge<'a>> {
        let vehicle_name = actor
            .attributes()
            .iter()
            .find(|attr| attr.id() == "role_vehicle_name")
            .unwrap()
            .value_string();
        info!("Detect a vehicle {vehicle_name}");
        let physics_control = actor.physics_control();
        let controller = VehicleController::from_physics_control(&physics_control, None);

        let publisher_velocity = z_session
            .declare_publisher(format!("{vehicle_name}/rt/vehicle/status/velocity_status"))
            .res()?;
        let speed = Arc::new(AtomicF32::new(0.0));

        // TODO: We can use default value here
        let current_ackermann_cmd =
            Arc::new(ArcSwap::from_pointee(AckermannControlCommand::default()));
        let cloned_cmd = current_ackermann_cmd.clone();
        let subscriber_control_cmd = z_session
            .declare_subscriber(format!("{vehicle_name}/rt/external/selected/control_cmd"))
            .callback_mut(move |sample| {
                let result: Result<AckermannControlCommand, _> =
                    cdr::deserialize_from(sample.payload.reader(), cdr::size::Infinite);
                let Ok(cmd) = result else {
                    return;
                };
                cloned_cmd.store(Arc::new(cmd));
            })
            .res()?;
        let _subscriber_gate_mode = z_session
            .declare_subscriber(format!("{vehicle_name}/rt/control/gate_mode_cmd"))
            .callback_mut(move |_| {
                // TODO
            })
            .res()?;
        let subscriber_gear_cmd = z_session
            .declare_subscriber(format!("{vehicle_name}/rt/external/selected/gear_cmd"))
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
            speed,
            controller,
            current_ackermann_cmd,
        })
    }

    fn pub_current_velocity(&mut self, stamp: &Time) -> Result<()> {
        //let transform = vehicle_actor.transform();
        let velocity = self.actor.velocity();
        //let angular_velocity = vehicle_actor.angular_velocity();
        //let accel = vehicle_actor.acceleration();
        let velocity_msg = VelocityReport {
            header: Header {
                // TODO: Use correct timestamp
                stamp: stamp.clone(),
                frame_id: String::from(""),
            },
            longitudinal_velocity: velocity.norm(),
            lateral_velocity: 0.0,
            // The heading rate is 1 deg to 0.00866, and the direction is reverse
            heading_rate: self
                .actor
                .get_wheel_steer_angle(VehicleWheelLocation::FL_Wheel)
                * -0.00866,
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
    fn step(&mut self, stamp: &Time, elapsed_sec: f64) -> Result<()> {
        self.pub_current_velocity(stamp)?;
        self.update_carla_control(elapsed_sec);
        Ok(())
    }
}

impl<'a> Drop for VehicleBridge<'a> {
    fn drop(&mut self) {
        info!("Remove vehicle {}", self.vehicle_name());
    }
}
