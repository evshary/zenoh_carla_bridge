from carla import Actor, Transform, Location, AttachmentType
import weakref
from ..utils import get_actor_bounding_extent

class LidarSensor(object):
    callback = None

    def __init__(self, actor: Actor, range: float = 50):
        extent = get_actor_bounding_extent(actor)
        bound_x = extent.x
        bound_y = extent.y
        bound_z = extent.z

        world = actor.get_world()
        trans = Transform(Location(x=+0.8 * bound_x, y=+0.0 * bound_y, z=1.3 * bound_z))
        bp = world.get_blueprint_library().find("sensor.lidar.ray_cast_semantic")
        bp.set_attribute("channels", "32")
        bp.set_attribute("points_per_second", "600000")
        bp.set_attribute("rotation_frequency", "10")
        bp.set_attribute("range", str(range))
        sensor = world.spawn_actor(
            bp,
            trans,
            attach_to=actor,
            attachment_type=AttachmentType.Rigid,
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
