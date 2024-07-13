import weakref

from carla import Actor, Location, Rotation, Transform


class LidarSensor(object):
    callback = None

    def __init__(self, actor: Actor, range: float = 50, sensor_name='lidar', trans=None):
        world = actor.get_world()
        bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
        bp.set_attribute('role_name', sensor_name)
        bp.set_attribute('range', str(100))
        bp.set_attribute('rotation_frequency', str(20))
        bp.set_attribute('channels', str(64))
        bp.set_attribute('upper_fov', str(10))
        bp.set_attribute('lower_fov', str(-30))
        bp.set_attribute('points_per_second', str(1200000))
        bp.set_attribute('atmosphere_attenuation_rate', str(0.004))
        bp.set_attribute('dropoff_general_rate', str(0.45))
        bp.set_attribute('dropoff_intensity_limit', str(0.8))
        bp.set_attribute('dropoff_zero_intensity', str(0.4))

        if trans is None:
            trans = Transform(Location(x=0.0, y=0.0, z=2.4), Rotation(pitch=0.0, roll=0.0, yaw=270.0))

        sensor = world.spawn_actor(
            bp,
            trans,
            attach_to=actor,
        )

        # We need a weak reference to self to avoid circular reference.
        weak_self = weakref.ref(self)
        sensor.listen(lambda image: LidarSensor._private_callback(weak_self, image))

        self.sensor = sensor
        self._parent = actor

    def __del__(self):
        self.sensor.destroy()

    def set_callback(self, callback):
        self.callback = callback

    @staticmethod
    def _private_callback(weak_self, data):
        # return if the parent no longer exists
        me = weak_self()
        if not me:
            return

        # Invoke callback
        if me.callback is not None:
            me.callback(data)
