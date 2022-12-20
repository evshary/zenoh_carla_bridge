use std::sync::Arc;
use std::thread;
use std::time::Duration;
use zenoh::prelude::sync::*;
use cdr::{CdrLe, Infinite};
use carla::client::{Vehicle, ActorBase};
use carla::rpc::VehicleWheelLocation;
use crate::autoware_type;

pub struct VehicleBridge {
    _vehicle_name: String,
    _actor: Vehicle,
}

impl VehicleBridge {
    pub fn new(z_session: Arc<Session>, name: String, actor: Vehicle) -> VehicleBridge {
        let publisher_velocity = z_session
            // TODO: Check whether Zenoh can receive the message
            .declare_publisher(name.clone()+"rt/vehicle/status/velocity_status")
            .res()
            .unwrap();
        let mut vehicle_actor = actor.clone();
        thread::spawn(move || loop {
            //let transform = vehicle_actor.transform();
            let velocity = vehicle_actor.velocity();
            //let angular_velocity = vehicle_actor.angular_velocity();
            //let accel = vehicle_actor.acceleration();
            let velocity_msg = autoware_type::CurrentVelocity {
                header: autoware_type::StdMsgsHeader {  // TODO: Use correct timestamp
                    ts: autoware_type::TimeStamp { sec: 0, nsec: 0},
                    frameid: String::from(""),
                },
                longitudinal_velocity: velocity.norm(),
                lateral_velocity: 0.0,
                heading_rate: vehicle_actor.get_wheel_steer_angle(VehicleWheelLocation::FL_Wheel) * -0.00866, // The heading rate is 1 deg to 0.00866, and the direction is reverse
            };
            let encoded = cdr::serialize::<_, _, CdrLe>(&velocity_msg, Infinite).unwrap();
            publisher_velocity.put(encoded).res().unwrap();
            // TODO: Check the published rate
            thread::sleep(Duration::from_millis(1000));
            //thread::sleep(Duration::from_millis(33)); // 30Hz
        });
        VehicleBridge { 
            _vehicle_name: name,
            _actor: actor,
        }
    }
}