import os

SIM_WORLD = ''
if os.environ.get('CARLA_MAP_NAME'):
    SIM_WORLD = os.environ['CARLA_MAP_NAME']
else:
    SIM_WORLD = 'Town03'
