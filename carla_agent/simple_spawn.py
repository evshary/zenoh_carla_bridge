import random

import carla

from simulation import config
from simulation.sensors import (
    GnssSensor,
    IMUSensor,
    LidarSensor,
    RgbCamera,
)

vehicle_list = [
    ('v1', '87.687683,145.671295,0.300000,0.000000,90.000053,0.000000'),
    ('v2', '92.109985,227.220001,0.300000,0.000000,-90.000298,0.000000')
]

def main():
    client = carla.Client("127.0.0.1", 2000)
    client.set_timeout(20.0)

    sim_world = client.load_world(config.SIM_WORLD)

    for (vehicle_name, position) in vehicle_list:
        # vehicles settings
        vehicles = sim_world.get_blueprint_library().filter("vehicle.tesla.model3")
        blueprint = random.choice(vehicles)
        blueprint.set_attribute('role_name', "autoware_"+vehicle_name)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)
        if blueprint.has_attribute('is_invincible'):
            blueprint.set_attribute('is_invincible', 'true')

        # spawn
        position = position.split(',')
        spawn_point = carla.Transform(carla.Location(x=float(position[0]), y=float(position[1]), z=float(position[2])), 
                                    carla.Rotation(pitch=float(position[3]), yaw=float(position[4]), roll=float(position[5])))
        vehicle_actor = sim_world.try_spawn_actor(blueprint, spawn_point)
        # Set sweep_wheel_collision
        physics_control = vehicle_actor.get_physics_control()
        physics_control.use_sweep_wheel_collision = True
        vehicle_actor.apply_physics_control(physics_control)

        # Setup sensors
        _gnss_sensor = GnssSensor(vehicle_actor, sensor_name='ublox')
        _imu_sensor = IMUSensor(vehicle_actor, sensor_name='tamagawa')
        _lidar_sensor = LidarSensor(vehicle_actor, sensor_name='top')
        _rgb_camera = RgbCamera(vehicle_actor, sensor_name='traffic_light')

    while True:
        sim_world.wait_for_tick()


if __name__ == '__main__':
    main()
