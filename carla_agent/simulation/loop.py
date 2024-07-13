import time

import carla
import pygame

from . import config
from .keyboard import KeyboardControl
from .ui import HUD
from .world import World


def game_loop(args, doc):
    pygame.init()
    if args.pygame:
        pygame.font.init()
    world = None
    original_settings = None

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(20.0)

        sim_world = client.load_world(config.SIM_WORLD)
        # sim_world = client.get_world()
        if args.sync:
            original_settings = sim_world.get_settings()
            settings = sim_world.get_settings()
            if not settings.synchronous_mode:
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = 0.05
            sim_world.apply_settings(settings)

            traffic_manager = client.get_trafficmanager()
            traffic_manager.set_synchronous_mode(True)

        if args.autopilot and not sim_world.get_settings().synchronous_mode:
            print('WARNING: You are currently in asynchronous mode and could ' 'experience some issues with the traffic simulation')

        if args.pygame:
            display = pygame.display.set_mode((args.width, args.height), pygame.HWSURFACE | pygame.DOUBLEBUF)
            display.fill((0, 0, 0))
            pygame.display.flip()

        hud = HUD(args.width, args.height, doc)
        world = World(sim_world, hud, args)
        if args.pygame:
            controller = KeyboardControl(world, args.autopilot)

        if args.sync:
            sim_world.tick()
        else:
            sim_world.wait_for_tick()

        if args.pygame:
            clock = pygame.time.Clock()
        while True:
            if args.sync:
                sim_world.tick()
                # 20 ticks for 1 simulated seconds, and tick every 0.1 real seconds
                # 1 simulated second = 2 real seconds
                time.sleep(0.1)
            if args.pygame:
                clock.tick_busy_loop(60)
                if controller.parse_events(client, world, clock, args.sync):
                    return
                world.tick(clock)
                world.render(display)
                pygame.display.flip()

    finally:
        if original_settings:
            sim_world.apply_settings(original_settings)

        if world and world.recording_enabled:
            client.stop_recorder()

        if world is not None:
            world.destroy()

        pygame.quit()
