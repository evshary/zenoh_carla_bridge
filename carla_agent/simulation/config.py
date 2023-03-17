import os
from carla import Transform, Location, Rotation

SIM_WORLD = ""
if os.environ.get('CARLA_MAP_NAME'):
    SIM_WORLD = os.environ['CARLA_MAP_NAME']
else:
    SIM_WORLD = "Town03"
INIT_POSE = Transform(Location(x=88.619980, y=226.905640, z=0.300000), Rotation(pitch=0.000000, yaw=90.000046, roll=0.000000))
