use std::sync::{atomic::Ordering, Arc};

use arc_swap::ArcSwap;
use atomic_float::AtomicF32;
use carla::{
    client::{ActorBase, Vehicle},
    rpc::{VehicleControl, VehicleWheelLocation},
};
use cdr::{CdrLe, Infinite};
use interp::{interp, InterpMode};
use zenoh::{
    pubsub::{Publisher, Subscriber},
    Session, Wait,
};
use zenoh_ros_type::{
    autoware_vehicle_msgs::{
        control_mode_report, gear_report, hazard_lights_report, turn_indicators_report,
        ControlModeReport, GearCommand, GearReport, HazardLightsReport, SteeringReport,
        TurnIndicatorsReport, VelocityReport,
    },
    builtin_interfaces::Time,
    std_msgs::Header,
    tier4_control_msgs::{gate_mode_data, GateMode},
    tier4_vehicle_msgs::{
        ActuationCommand, ActuationCommandStamped, ActuationStatus, ActuationStatusStamped,
    },
};

use super::actor_bridge::{ActorBridge, BridgeType};
use crate::{
    autoware::Autoware,
    error::{BridgeError, Result},
    put_with_attachment, utils,
};

pub struct VehicleBridge<'a> {
    vehicle_name: String,
    actor: Vehicle,
    _subscriber_actuation_cmd: Subscriber<()>,
    _subscriber_gear_cmd: Subscriber<()>,
    _subscriber_gate_mode: Subscriber<()>,
    publisher_actuation: Publisher<'a>,
    publisher_velocity: Publisher<'a>,
    publisher_steer: Publisher<'a>,
    publisher_gear: Publisher<'a>,
    publisher_control: Publisher<'a>,
    publisher_turnindicator: Publisher<'a>,
    publisher_hazardlight: Publisher<'a>,
    velocity: Arc<AtomicF32>,
    current_actuation_cmd: Arc<ArcSwap<ActuationCommandStamped>>,
    current_gear: Arc<ArcSwap<u8>>,
    current_gate_mode: Arc<ArcSwap<GateMode>>,
    attachment: Vec<u8>,
    mode: crate::Mode,
}

impl<'a> VehicleBridge<'a> {
    pub fn get_bridge_type(actor: Vehicle) -> Result<BridgeType> {
        let mut vehicle_name = actor
            .attributes()
            .iter()
            .find(|attr| attr.id() == "role_name")
            .ok_or(BridgeError::CarlaIssue(
                "Unable to find role_name in the vehicle",
            ))?
            .value_string();

        // Remove "autoware_" in role name
        if !vehicle_name.starts_with("autoware_") {
            return Err(BridgeError::Npc {
                npc_role_name: vehicle_name,
            });
        }
        vehicle_name = vehicle_name.replace("autoware_", "");

        log::info!("Detect a vehicle {vehicle_name}");

        Ok(BridgeType::Vehicle(vehicle_name))
    }

    pub fn new(
        z_session: Arc<Session>,
        actor: Vehicle,
        bridge_type: BridgeType,
        autoware: &Autoware,
    ) -> Result<VehicleBridge<'a>> {
        let vehicle_name = match bridge_type {
            BridgeType::Vehicle(v) => v,
            _ => panic!("Should never happen!"),
        };

        // The actuation status is only used in accel_brake_map_calibrator; Autoware does not subscribe to it.
        let publisher_actuation = z_session
            .declare_publisher(autoware.topic_actuation_status.clone())
            .wait()?;
        let publisher_velocity = z_session
            .declare_publisher(autoware.topic_velocity_status.clone())
            .wait()?;
        let publisher_steer = z_session
            .declare_publisher(autoware.topic_steering_status.clone())
            .wait()?;
        let publisher_gear = z_session
            .declare_publisher(autoware.topic_gear_status.clone())
            .wait()?;
        let publisher_control = z_session
            .declare_publisher(autoware.topic_control_mode.clone())
            .wait()?;
        let publisher_turnindicator = z_session
            .declare_publisher(autoware.topic_turn_indicators_status.clone())
            .wait()?;
        let publisher_hazardlight = z_session
            .declare_publisher(autoware.topic_hazard_lights_status.clone())
            .wait()?;
        let velocity = Arc::new(AtomicF32::new(0.0));

        // Using actuation instead of Ackermann to prevent mismatch with Autoware's speed logic and CARLA's motion constraints
        let current_actuation_cmd = Arc::new(ArcSwap::from_pointee(ActuationCommandStamped {
            header: Header {
                stamp: Time { sec: 0, nanosec: 0 },
                frame_id: "".to_string(),
            },
            actuation: ActuationCommand {
                accel_cmd: 0.0,
                brake_cmd: 0.0,
                steer_cmd: 0.0,
            },
        }));
        let cloned_cmd = current_actuation_cmd.clone();
        let subscriber_actuation_cmd = z_session
            .declare_subscriber(autoware.topic_actuation_cmd.clone())
            .callback_mut(move |sample| {
                let result: Result<ActuationCommandStamped, _> =
                    cdr::deserialize_from(sample.payload().reader(), cdr::size::Infinite);
                let Ok(cmd) = result else {
                    log::error!("Unable to parse data from /control/command/actuation_cmd");
                    return;
                };
                cloned_cmd.store(Arc::new(cmd));
            })
            .wait()?;
        let current_gear = Arc::new(ArcSwap::from_pointee(gear_report::NONE));
        let cloned_gear = current_gear.clone();
        let subscriber_gear_cmd = z_session
            .declare_subscriber(autoware.topic_gear_cmd.clone())
            .callback_mut(move |sample| {
                let result: Result<GearCommand, _> =
                    cdr::deserialize_from(sample.payload().reader(), cdr::size::Infinite);
                let Ok(cmd) = result else {
                    log::error!("Unable to parse data from /control/command/gear_cmd");
                    return;
                };
                cloned_gear.store(Arc::new(cmd.command));
            })
            .wait()?;
        let current_gate_mode = Arc::new(ArcSwap::from_pointee(GateMode {
            data: gate_mode_data::AUTO,
        }));
        let cloned_gate_mode = current_gate_mode.clone();
        let subscriber_gate_mode = z_session
            .declare_subscriber(autoware.topic_current_gate_mode.clone())
            .callback_mut(move |sample| {
                let result: Result<GateMode, _> =
                    cdr::deserialize_from(sample.payload().reader(), cdr::size::Infinite);
                let Ok(mode) = result else {
                    log::error!("Unable to parse data from /control/current_gate_mode");
                    return;
                };
                cloned_gate_mode.store(Arc::new(mode));
            })
            .wait()?;
        let _subscriber_turnindicator = z_session
            .declare_subscriber(autoware.topic_turn_indicators_cmd.clone())
            .callback_mut(move |_sample| {
                // TODO: Not support yet
            })
            .wait()?;
        let _subscriber_hazardlight = z_session
            .declare_subscriber(autoware.topic_hazard_lights_cmd.clone())
            .callback_mut(move |_sample| {
                // TODO: Not support yet
            })
            .wait()?;

        // Generate rmw_zenoh-compatible attachment
        let attachment = utils::generate_attachment();

        Ok(VehicleBridge {
            vehicle_name,
            actor,
            _subscriber_actuation_cmd: subscriber_actuation_cmd,
            _subscriber_gear_cmd: subscriber_gear_cmd,
            _subscriber_gate_mode: subscriber_gate_mode,
            publisher_actuation,
            publisher_velocity,
            publisher_steer,
            publisher_gear,
            publisher_control,
            publisher_turnindicator,
            publisher_hazardlight,
            velocity,
            current_actuation_cmd,
            current_gear,
            current_gate_mode,
            attachment,
            mode: autoware.mode.clone(),
        })
    }

    fn pub_current_actuation(&mut self, timestamp: f64) -> Result<()> {
        let control = self.actor.control();
        let mut header = utils::create_ros_header(Some(timestamp));
        header.frame_id = String::from("base_link");
        let actuation_msg = ActuationStatusStamped {
            header,
            status: ActuationStatus {
                accel_status: control.throttle as f64,
                brake_status: control.brake as f64,
                steer_status: -control.steer as f64,
            },
        };
        log::debug!(
            "Carla => Autoware: accel_status={:.3}, brake_status={:.3}, steer_status={:.3}",
            control.throttle,
            control.brake,
            -control.steer
        );
        let encoded = cdr::serialize::<_, _, CdrLe>(&actuation_msg, Infinite)?;
        put_with_attachment!(
            self.publisher_actuation,
            encoded,
            self.attachment,
            self.mode
        )?;
        Ok(())
    }

    fn pub_current_velocity(&mut self, timestamp: f64) -> Result<()> {
        //let transform = vehicle_actor.transform();
        let velocity = self.actor.velocity();
        //let angular_velocity = vehicle_actor.angular_velocity();
        //let accel = vehicle_actor.acceleration();
        let mut header = utils::create_ros_header(Some(timestamp));
        header.frame_id = String::from("base_link");
        let velocity_msg = VelocityReport {
            header,
            // Since the velocity report from Carla is always positive, we need to check reverse.
            longitudinal_velocity: if self.actor.control().reverse {
                -velocity.norm()
            } else {
                velocity.norm()
            },
            lateral_velocity: 0.0,
            heading_rate: self
                .actor
                .wheel_steer_angle(VehicleWheelLocation::FL_Wheel)
                .to_radians()
                * -1.0,
        };
        log::debug!(
            "Carla => Autoware: current velocity: {}",
            velocity_msg.longitudinal_velocity
        );
        let encoded = cdr::serialize::<_, _, CdrLe>(&velocity_msg, Infinite)?;
        put_with_attachment!(self.publisher_velocity, encoded, self.attachment, self.mode)?;
        self.velocity
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
        put_with_attachment!(self.publisher_steer, encoded, self.attachment, self.mode)?;
        Ok(())
    }

    fn pub_current_gear(&mut self, timestamp: f64) -> Result<()> {
        let gear_msg = GearReport {
            stamp: Time {
                sec: timestamp.floor() as i32,
                nanosec: (timestamp.fract() * 1_000_000_000_f64) as u32,
            },
            report: **self.current_gear.load(),
        };
        let encoded = cdr::serialize::<_, _, CdrLe>(&gear_msg, Infinite)?;
        put_with_attachment!(self.publisher_gear, encoded, self.attachment, self.mode)?;
        Ok(())
    }

    fn pub_current_control(&mut self, timestamp: f64) -> Result<()> {
        let mode = if self.current_gate_mode.load().data == gate_mode_data::AUTO {
            control_mode_report::AUTONOMOUS
        } else {
            control_mode_report::MANUAL
        };
        let control_msg = ControlModeReport {
            stamp: Time {
                sec: timestamp.floor() as i32,
                nanosec: (timestamp.fract() * 1_000_000_000_f64) as u32,
            },
            mode,
        };
        let encoded = cdr::serialize::<_, _, CdrLe>(&control_msg, Infinite)?;
        put_with_attachment!(self.publisher_control, encoded, self.attachment, self.mode)?;
        Ok(())
    }

    fn pub_current_indicator(&mut self, timestamp: f64) -> Result<()> {
        // TODO: Not support yet
        let turnindicator_msg = TurnIndicatorsReport {
            stamp: Time {
                sec: timestamp.floor() as i32,
                nanosec: (timestamp.fract() * 1_000_000_000_f64) as u32,
            },
            report: turn_indicators_report::DISABLE,
        };
        let encoded = cdr::serialize::<_, _, CdrLe>(&turnindicator_msg, Infinite)?;
        put_with_attachment!(
            self.publisher_turnindicator,
            encoded,
            self.attachment,
            self.mode
        )?;
        Ok(())
    }

    fn pub_hazard_light(&mut self, timestamp: f64) -> Result<()> {
        // TODO: Not support yet
        let hazardlight_msg = HazardLightsReport {
            stamp: Time {
                sec: timestamp.floor() as i32,
                nanosec: (timestamp.fract() * 1_000_000_000_f64) as u32,
            },
            report: hazard_lights_report::DISABLE,
        };
        let encoded = cdr::serialize::<_, _, CdrLe>(&hazardlight_msg, Infinite)?;
        put_with_attachment!(
            self.publisher_hazardlight,
            encoded,
            self.attachment,
            self.mode
        )?;
        Ok(())
    }

    fn update_carla_control(&mut self) {
        let ActuationCommandStamped {
            actuation:
                ActuationCommand {
                    mut accel_cmd,
                    mut brake_cmd,
                    mut steer_cmd,
                },
            ..
        } = **self.current_actuation_cmd.load();

        log::debug!(
            "Autoware => Bridge: accel_cmd={:.3}, brake_cmd={:.3}, steer_cmd={:.3}",
            accel_cmd,
            brake_cmd,
            steer_cmd,
        );

        // Default states
        let mut reverse = false;
        let mut hand_brake = false;

        match **self.current_gear.load() {
            gear_report::DRIVE => { /* Do nothing */ }
            gear_report::REVERSE => {
                /* Set reverse to true for reverse gear */
                reverse = true;
            }
            gear_report::PARK => {
                /* Force the vehicle to stop */
                accel_cmd = 0.0;
                brake_cmd = 0.0;
                steer_cmd = 0.0;
                hand_brake = true;
            }
            _ => { /* Do nothing */ }
        };

        // Convert steer_cmd base on steering curve and speed of the vehicle
        let steering_curve = self.actor.physics_control().steering_curve;
        let v_x: Vec<f32> = steering_curve.iter().map(|v| v[0]).collect();
        let v_y: Vec<f32> = steering_curve.iter().map(|v| v[1]).collect();

        let current_speed = self.actor.velocity().x.abs();
        let max_steer_ratio = interp(&v_x, &v_y, current_speed, &InterpMode::default());

        let max_steer_angle_rad = self.actor.physics_control().wheels[0]
            .max_steer_angle
            .to_radians();
        let steer = -steer_cmd as f32 * max_steer_ratio * max_steer_angle_rad;

        self.actor.apply_control(&VehicleControl {
            throttle: accel_cmd as f32,
            steer,
            brake: brake_cmd as f32,
            hand_brake,
            reverse,
            manual_gear_shift: false,
            gear: 0,
        });
        log::debug!(
            "Bridge => Carla: throttle={:.3}, steer={:.3}, brake={:.3}, hand_brake={}, reverse={}",
            accel_cmd as f32,
            steer,
            brake_cmd as f32,
            hand_brake,
            reverse,
        );
    }

    pub fn vehicle_name(&self) -> &str {
        &self.vehicle_name
    }
}

impl ActorBridge for VehicleBridge<'_> {
    fn step(&mut self, timestamp: f64) -> Result<()> {
        self.pub_current_actuation(timestamp)?;
        self.pub_current_velocity(timestamp)?;
        self.pub_current_steer(timestamp)?;
        self.pub_current_gear(timestamp)?;
        self.pub_current_control(timestamp)?;
        self.pub_current_indicator(timestamp)?;
        self.pub_hazard_light(timestamp)?;
        self.update_carla_control();
        Ok(())
    }
}

impl Drop for VehicleBridge<'_> {
    fn drop(&mut self) {
        log::info!("Remove vehicle name {}", self.vehicle_name());
    }
}
