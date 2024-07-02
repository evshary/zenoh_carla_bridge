import math
import weakref

import carla


class IMUSensor(object):
    def __init__(self, parent_actor, sensor_name = 'imu', trans = None):
        self.sensor = None
        self._parent = parent_actor
        self.accelerometer = (0.0, 0.0, 0.0)
        self.gyroscope = (0.0, 0.0, 0.0)
        self.compass = 0.0

        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.imu')
        bp.set_attribute('role_name', sensor_name)
        bp.set_attribute('noise_accel_stddev_x', str(0.0))
        bp.set_attribute('noise_accel_stddev_y', str(0.0))
        bp.set_attribute('noise_accel_stddev_z', str(0.0))
        bp.set_attribute('noise_gyro_stddev_x', str(0.0))
        bp.set_attribute('noise_gyro_stddev_y', str(0.0))
        bp.set_attribute('noise_gyro_stddev_z', str(0.0))

        if trans is None:
            trans = carla.Transform(carla.Location(x=0.0, y=0.0, z=2.4),
                                    carla.Rotation(roll=0.0, pitch=0.0, yaw=270.0))

        self.sensor = world.spawn_actor(
            bp,
            trans,
            attach_to=self._parent
        )

        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda sensor_data: IMUSensor._IMU_callback(weak_self, sensor_data))

    @staticmethod
    def _IMU_callback(weak_self, sensor_data):
        self = weak_self()
        if not self:
            return
        limits = (-99.9, 99.9)
        self.accelerometer = (
            max(limits[0], min(limits[1], sensor_data.accelerometer.x)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.y)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.z)))
        self.gyroscope = (
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.x))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.y))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.z))))
        self.compass = math.degrees(sensor_data.compass)
