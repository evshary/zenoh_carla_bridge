use super::actor_bridge::ActorBridge;
use crate::{
    error::{Error, Result},
    utils,
};
use arc_swap::ArcSwap;
use atomic_float::AtomicF32;
use carla::{
    client::{ActorBase, Vehicle},
    rpc::{VehicleAckermannControl, VehicleWheelLocation},
};
use cdr::{CdrLe, Infinite};
use log::{debug, info};
use std::sync::{atomic::Ordering, Arc};
use zenoh::{prelude::sync::*, publication::Publisher, subscriber::Subscriber};
use zenoh_ros_type::{
    autoware_auto_control_msgs::{
        AckermannControlCommand, AckermannLateralCommand, LongitudinalCommand,
    },
    autoware_auto_vehicle_msgs::{
        control_mode_report, gear_report, hazard_lights_report, turn_indicators_report,
        ControlModeReport, GearCommand, GearReport, HazardLightsReport, SteeringReport,
        TurnIndicatorsReport, VelocityReport,
    },
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
    publisher_turnindicator: Publisher<'a>,
    publisher_hazardlight: Publisher<'a>,
    speed: Arc<AtomicF32>,
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
        let publisher_turnindicator = z_session
            .declare_publisher(format!(
                "{vehicle_name}/rt/vehicle/status/turn_indicators_status"
            ))
            .res()?;
        let publisher_hazardlight = z_session
            .declare_publisher(format!(
                "{vehicle_name}/rt/vehicle/status/hazard_lights_status"
            ))
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

        let _subscriber_turnindicator = z_session
            .declare_subscriber(format!(
                "{vehicle_name}/rt/control/command/turn_indicators_cmd"
            ))
            .callback_mut(move |_sample| {
                // TODO: Not support yet
            })
            .res()?;
        let _subscriber_hazardlight = z_session
            .declare_subscriber(format!(
                "{vehicle_name}/rt/control/command/hazard_lights_cmd"
            ))
            .callback_mut(move |_sample| {
                // TODO: Not support yet
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
            publisher_turnindicator,
            publisher_hazardlight,
            speed,
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
                .wheel_steer_angle(VehicleWheelLocation::FL_Wheel)
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
                nanosec: (timestamp.fract() * 1_000_000_000_f64) as u32,
            },
            steering_tire_angle: self
                .actor
                .wheel_steer_angle(VehicleWheelLocation::FL_Wheel)
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
                nanosec: (timestamp.fract() * 1_000_000_000_f64) as u32,
            },
            report: if self.actor.control().reverse { 20 } else { 2 }, /* TODO: Use enum (20: reverse, 2: drive) */
        };
        let encoded = cdr::serialize::<_, _, CdrLe>(&gear_msg, Infinite)?;
        self.publisher_gear.put(encoded).res()?;
        Ok(())
    }

    fn pub_current_control(&mut self, timestamp: f64) -> Result<()> {
        let control_msg = ControlModeReport {
            stamp: Time {
                sec: timestamp.floor() as i32,
                nanosec: (timestamp.fract() * 1_000_000_000_f64) as u32,
            },
            mode: 1, /* 1: AUTONOMOUS, 4: MANUAL. TODO: Now we don't have any way to switch these two modes. */
        };
        let encoded = cdr::serialize::<_, _, CdrLe>(&control_msg, Infinite)?;
        self.publisher_control.put(encoded).res()?;
        Ok(())
    }

    fn pub_current_indicator(&mut self, timestamp: f64) -> Result<()> {
        // TODO: Not support yet
        let turnindicator_msg = TurnIndicatorsReport {
            stamp: Time {
                sec: timestamp.floor() as i32,
                nanosec: (timestamp.fract() * 1000_000_000_f64) as u32,
            },
            report: turn_indicators_report::DISABLE,
        };
        let encoded = cdr::serialize::<_, _, CdrLe>(&turnindicator_msg, Infinite)?;
        self.publisher_turnindicator.put(encoded).res()?;
        Ok(())
    }

    fn pub_hazard_light(&mut self, timestamp: f64) -> Result<()> {
        // TODO: Not support yet
        let hazardlight_msg = HazardLightsReport {
            stamp: Time {
                sec: timestamp.floor() as i32,
                nanosec: (timestamp.fract() * 1000_000_000_f64) as u32,
            },
            report: hazard_lights_report::DISABLE,
        };
        let encoded = cdr::serialize::<_, _, CdrLe>(&hazardlight_msg, Infinite)?;
        self.publisher_hazardlight.put(encoded).res()?;
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
                    jerk,
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

        let steer = {
            let max_steer_angle = 69.999;
            (-steering_tire_angle.to_degrees() / max_steer_angle).clamp(-1.0, 1.0)
        };

        // Compute the steering speed
        let steer_speed = if steering_tire_angle.to_degrees().abs() < 3.0 {
            0.0
        } else if steering_tire_angle.to_degrees() > 0.0 {
            0.1
        } else {
            -0.1
        };

        self.actor
            .apply_ackermann_control(&VehicleAckermannControl {
                steer,
                steer_speed,
                speed,
                acceleration,
                jerk,
            });

        debug!(
            "Autoware => Carla: elapse_sec:{} current_speed:{} pitch_radians:{}",
            elapsed_sec, current_speed, pitch_radians
        );
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
        self.pub_current_indicator(timestamp)?;
        self.pub_hazard_light(timestamp)?;
        self.update_carla_control(elapsed_sec);
        Ok(())
    }
}

impl<'a> Drop for VehicleBridge<'a> {
    fn drop(&mut self) {
        info!("Remove vehicle name {}", self.vehicle_name());
    }
}

