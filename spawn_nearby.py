
'''
Stuff I need
1. Car extrinsics
2. Camera extrinsics wrt 1 car
3. Camera intrinsics
4. Bounding box for both cars wrt each car's origin
'''
import glob
import os
import sys
import time

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import ipdb
st = ipdb.set_trace
import argparse
import logging
import random
import numpy as  np
def pts_dist(locsi, locsj):
    dist = (locsi.x - locsj.x)**2 + (locsi.y - locsj.y)**2 + (locsi.z - locsj.z)**2
    return np.sqrt(dist)

def main():
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-n', '--number-of-vehicles',
        metavar='N',
        default=1,
        type=int,
        help='number of vehicles (default: 10)')
    argparser.add_argument(
        '-w', '--number-of-walkers',
        metavar='W',
        default=50,
        type=int,
        help='number of walkers (default: 50)')
    argparser.add_argument(
        '--safe',
        action='store_true',
        help='avoid spawning vehicles prone to accidents')
    argparser.add_argument(
        '--filterv',
        metavar='PATTERN',
        default='vehicle.*',
        help='vehicles filter (default: "vehicle.*")')
    argparser.add_argument(
        '--filterw',
        metavar='PATTERN',
        default='walker.pedestrian.*',
        help='pedestrians filter (default: "walker.pedestrian.*")')
    args = argparser.parse_args()

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    vehicles_list = []
    walkers_list = []
    all_id = []
    client = carla.Client(args.host, args.port)
    client.set_timeout(5.0)


    world = client.get_world()
    blueprints = world.get_blueprint_library().filter(args.filterv)
    blueprintsWalkers = world.get_blueprint_library().filter(args.filterw)

    spawn_points = world.get_map().get_spawn_points()
    number_of_spawn_points = len(spawn_points)

    
    # @todo cannot import these directly.
    SpawnActor = carla.command.SpawnActor
    SetAutopilot = carla.command.SetAutopilot
    FutureActor = carla.command.FutureActor

    # --------------
    # Spawn vehicles
    # --------------
    batch = []
    random.shuffle(spawn_points)
    locs = [s.location for s in spawn_points]
    
    transform = spawn_points[0]

    blueprint = random.choice(blueprints)
    if blueprint.has_attribute('color'):
        color = random.choice(blueprint.get_attribute('color').recommended_values)
        blueprint.set_attribute('color', color)
    if blueprint.has_attribute('driver_id'):
        driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
        blueprint.set_attribute('driver_id', driver_id)
    blueprint.set_attribute('role_name', 'autopilot')
    batch.append(SpawnActor(blueprint, transform).then(SetAutopilot(FutureActor, False)))

    twoCarCase = False
    if twoCarCase:
        closest_transform = None
        mindist = 100000
        for j in range(1, number_of_spawn_points):
            if pts_dist(locs[0], locs[j]) < mindist:
                mindist = pts_dist(locs[0], locs[j])
                closest_transform = spawn_points[j]
        print("Closest spawn points are : ", mindist)

        blueprint = random.choice(blueprints)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)
        blueprint.set_attribute('role_name', 'autopilot')
        batch.append(SpawnActor(blueprint, closest_transform).then(SetAutopilot(FutureActor, False)))

    multiCarCase = True
    if multiCarCase:
        closest_transforms = []
        mindist = 13
        for j in range(1, number_of_spawn_points):
            if pts_dist(locs[0], locs[j]) <= mindist:
                closest_transforms.append(spawn_points[j])
        print("Number of Closest spawn points are : ", len(closest_transforms))

        for transform in closest_transforms:
            blueprint = random.choice(blueprints)
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            if blueprint.has_attribute('driver_id'):
                driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                blueprint.set_attribute('driver_id', driver_id)
            blueprint.set_attribute('role_name', 'autopilot')
            batch.append(SpawnActor(blueprint, transform).then(SetAutopilot(FutureActor, False)))


    for response in client.apply_batch_sync(batch):
        if response.error:
            logging.error(response.error)
        else:
            vehicles_list.append(response.actor_id)

    # wait for a tick to ensure client receives the last transform of the walkers we have just created
    world.wait_for_tick()


    print('spawned %d vehicles, press Ctrl+C to exit.' % (len(vehicles_list)))

    while True:
        world.wait_for_tick()


if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
