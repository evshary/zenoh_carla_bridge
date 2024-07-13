import weakref

import carla


class LaneInvasionSensor(object):
    def __init__(self, parent_actor, hud, sensor_name='lane_invasion', trans=None):
        self.sensor = None

        # If the spawn object is not a vehicle, we cannot use the Lane Invasion Sensor
        if parent_actor.type_id.startswith('vehicle.'):
            self._parent = parent_actor
            self.hud = hud

            world = self._parent.get_world()
            bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
            bp.set_attribute('role_name', sensor_name)

            if trans is None:
                trans = carla.Transform()

            self.sensor = world.spawn_actor(bp, trans, attach_to=self._parent)

            # We need to pass the lambda a weak reference to self to avoid circular
            # reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda event: LaneInvasionSensor._on_invasion(weak_self, event))

    @staticmethod
    def _on_invasion(weak_self, event):
        self = weak_self()
        if not self:
            return
        lane_types = set(x.type for x in event.crossed_lane_markings)
        text = ['%r' % str(x).split()[-1] for x in lane_types]
        self.hud.notification('Crossed line %s' % ' and '.join(text))
