"""Load the configured world once so per-vehicle agents can `main.py --attach`
to it without each reloading (a reload destroys every other vehicle's actors)."""
import sys

import carla

from simulation import config

host = sys.argv[1] if len(sys.argv) > 1 else '127.0.0.1'
client = carla.Client(host, 2000)
client.set_timeout(60.0)
client.load_world(config.SIM_WORLD)
print('world loaded:', config.SIM_WORLD)
