import collections
import math
import weakref

import carla

from ..utils import get_actor_display_name


class CollisionSensor(object):
    def __init__(self, parent_actor, hud, sensor_name = 'collision', trans = None):
        self.sensor = None
        self.history = []
        self._parent = parent_actor
        self.hud = hud

        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        bp.set_attribute('role_name', sensor_name)

        if trans is None:
            trans = carla.Transform()

        self.sensor = world.spawn_actor(
            bp,
            trans,
            attach_to=self._parent
        )

        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

    def get_collision_history(self):
        history = collections.defaultdict(int)
        for frame, intensity in self.history:
            history[frame] += intensity
        return history

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return
        actor_type = get_actor_display_name(event.other_actor)
        self.hud.notification('Collision with %r' % actor_type)
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
        self.history.append((event.frame, intensity))
        if len(self.history) > 4000:
            self.history.pop(0)
