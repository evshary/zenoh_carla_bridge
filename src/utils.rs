use crate::error::Result;
use carla::geom::BoundingBox;
use ndarray::Array2;
use r2r::{
    geometry_msgs::msg::{Point, Point32, Pose, PoseWithCovariance, Quaternion, Vector3},
    moveit_msgs::msg::OrientedBoundingBox,
    std_msgs::msg::Header,
    Clock, ClockType,
};

pub fn is_bigendian() -> bool {
    cfg!(target_endian = "big")
}

pub fn create_ros_header() -> Result<Header> {
    let mut clock = Clock::create(ClockType::RosTime)?;
    let duration = clock.get_now()?;
    let time = Clock::to_builtin_time(&duration);
    Ok(Header {
        stamp: time,
        frame_id: "".to_string(),
    })
}

pub fn identity_matrix(size: usize) -> Array2<f64> {
    Array2::from_diag_elem(size, 1.0)
}

pub trait ToRosType<T> {
    fn to_ros_type(&self) -> T;
}

impl ToRosType<Quaternion> for nalgebra::UnitQuaternion<f64> {
    fn to_ros_type(&self) -> Quaternion {
        Quaternion {
            x: self.i,
            y: self.j,
            z: self.k,
            w: self.w,
        }
    }
}

impl ToRosType<Quaternion> for nalgebra::UnitQuaternion<f32> {
    fn to_ros_type(&self) -> Quaternion {
        let val: nalgebra::UnitQuaternion<f64> = nalgebra::convert_ref(self);
        val.to_ros_type()
    }
}

impl ToRosType<Pose> for nalgebra::Isometry3<f64> {
    fn to_ros_type(&self) -> Pose {
        let nalgebra::Isometry3 {
            rotation,
            translation,
        } = self;

        Pose {
            position: Point {
                x: translation.x,
                y: translation.y,
                z: translation.z,
            },
            orientation: Quaternion {
                x: rotation.i,
                y: rotation.j,
                z: rotation.k,
                w: rotation.w,
            },
        }
    }
}

impl ToRosType<Pose> for nalgebra::Isometry3<f32> {
    fn to_ros_type(&self) -> Pose {
        let val: nalgebra::Isometry3<f64> = nalgebra::convert_ref(self);
        val.to_ros_type()
    }
}

impl ToRosType<PoseWithCovariance> for nalgebra::Isometry3<f64> {
    fn to_ros_type(&self) -> PoseWithCovariance {
        PoseWithCovariance {
            pose: self.to_ros_type(),
            covariance: identity_matrix(6).into_raw_vec(),
        }
    }
}

impl ToRosType<PoseWithCovariance> for nalgebra::Isometry3<f32> {
    fn to_ros_type(&self) -> PoseWithCovariance {
        let val: nalgebra::Isometry3<f64> = nalgebra::convert_ref(self);
        val.to_ros_type()
    }
}

impl ToRosType<Vector3> for nalgebra::Vector3<f64> {
    fn to_ros_type(&self) -> Vector3 {
        Vector3 {
            x: self.x,
            y: self.y,
            z: self.z,
        }
    }
}

impl ToRosType<Vector3> for nalgebra::Vector3<f32> {
    fn to_ros_type(&self) -> Vector3 {
        let val: nalgebra::Vector3<f64> = nalgebra::convert_ref(self);
        val.to_ros_type()
    }
}

impl ToRosType<Point32> for nalgebra::Vector3<f32> {
    fn to_ros_type(&self) -> Point32 {
        Point32 {
            x: self.x,
            y: self.y,
            z: self.z,
        }
    }
}

impl ToRosType<OrientedBoundingBox> for BoundingBox<f32> {
    fn to_ros_type(&self) -> OrientedBoundingBox {
        OrientedBoundingBox {
            pose: self.transform.to_ros_type(),
            extents: self.extent.to_ros_type(),
        }
    }
}
