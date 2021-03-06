import time
import random
import glob
import argparse
import logging
import sys
import os
import uuid
import copy
import numpy as np
import carla_utils
import pickle
import dynamic_weather as weather
try:
    import queue
except ImportError:
    import Queue as queue
    
try:
    sys.path.append(glob.glob('../../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla
import image_converter

from carla_sync_mode import CarlaSyncMode

'''
camR ranges: 8-14, -5,5, 1-3
'''
class CarlaMultiviewRunner():
    def __init__(self, blueprint_num):
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

        self.vehicle_blueprint_number = blueprint_num
        self.base_path = "_carla_multiview_single_vehicle_multiple_camRs_far_camera"
        self.host = "127.0.0.1"
        self.port = 2000
        self.number_of_episodes = 1
        self.filterv = "vehicle.*"
        self.num_vehicles = 1
        self.sensor_names = ['sensor.camera.rgb', 'sensor.camera.depth']
        self.num_types_of_sensors = len(self.sensor_names)
        self.num_datapoints_per_episode = 1
        self.min_frame_difference = 20
        self.randomize_each_camera = True

        self.should_randomize_camR = False
        self.camR_sensor_indices = []
        if self.should_randomize_camR:
            self.camR_Xrange = [5.0, 7]
            self.camR_Yrange = [-1, 3]
            self.camR_Zrange = [1, 3]

        # Camera specific params
        self.image_size_x = 256
        self.image_size_y = 256
        self.fov = 110
        self.focal = self.image_size_x/(2.0*np.tan(self.fov*np.pi/360.0))
        self.fps = 10 

        self.calculate_camera_locations()

    def calculate_camera_locations(self):

        # Position specific params
        self.nearest_x = 2.0
        self.dis_to_center = 20.0#5.0
        self.height = 0.65
        self.d_height = 1.0
        self.half_sqrt2 = np.sqrt(2)/2

        # Initilize camera positions, rotations and intrinsic matrices.
        center = np.array([self.nearest_x+self.dis_to_center, 0.0, self.height])
                
        self.positions = [] # [x,y,z]
        self.rotations = [] # [pitch, yaw, roll]

        # self.positions.append([3.5, 0.0, 1.0]) #CamR
        # self.rotations.append([-180.0, 0, 180])  #CamR

        # self.positions.append([5.0, 1.0, 1.0]) #CamR
        # self.rotations.append([-180.0, 0, 180])  #CamR


        # self.positions.append([-4.5, 1.5, 2.5])
        # self.rotations.append([8.0, -20, 0])

        # self.positions.append([4.0, 0.5, 1.5]) #check y val
        # self.rotations.append([0, 180, 0])

        # self.positions.append([-3.5, 1.0, 2.5])
        # self.rotations.append([-40, 0, 0])

        # self.positions.append([-1.5, 2.0, 3.0])
        # self.rotations.append([-36.0, -56.0, 0])

        # self.positions.append([1.5, -2.0, 3.0])
        # self.rotations.append([-26.0, -250.0, 0])

        # self.positions.append([0.5, -1.5, 4.8])
        # self.rotations.append([-90.0, 0, 0])
        
        radius_orig = 8
        if self.randomize_each_camera:
            radius = radius_orig + np.random.randint(-4,4)
        else:
            radius = radius_orig
        # Place the front camera first
        self.positions.append([radius, 0, 1])
        self.rotations.append([0, -180, 0])
        self.num_camRs = 1 # Number of camR candidates

        # These will be our camR candidates
        for yaw in range(40, 320, 35):
            if self.randomize_each_camera:
                radius = radius_orig + np.random.randint(-4,4)
            else:
                radius = radius_orig

            if yaw == 180:
                continue
            self.num_camRs += 1
            yaw_rads = np.radians(yaw)
            zcoord = 1
            xcoord = -radius*np.cos(yaw_rads)
            ycoord = radius*np.sin(yaw_rads)
            print("camR params are: ", xcoord, ycoord, zcoord, yaw)
            self.positions.append([xcoord, ycoord, zcoord])
            self.rotations.append([0, -yaw, 0])
        
        # Elevated cameras
        for yaw in range(40, 320, 35):
            if self.randomize_each_camera:
                radius = radius_orig + np.random.randint(-4,4)
            else:
                radius = radius_orig
            yaw_rads = np.radians(yaw)
            zcoord = 3.0
            xcoord = -radius*np.cos(yaw_rads)
            ycoord = radius*np.sin(yaw_rads)
            print("camera params are: ", xcoord, ycoord, zcoord, yaw)
            self.positions.append([xcoord, ycoord, zcoord])
            self.rotations.append([-40, -yaw, 0])

        # Directly overhead camera
        if self.randomize_each_camera:
            radius = radius_orig + np.random.randint(-4,4)
        else:
            radius = radius_orig
        self.positions.append([0, 0, radius])
        self.rotations.append([-90, 0, 0])
        
        self.num_locations_per_vehicle = len(self.positions) # Number of (single type) cameras on each vehicle
        print(self.positions)
        print(self.rotations)
        # Focal length is negative because of left handed system.
        self.K = np.array([[-self.focal, 0.0, self.image_size_x/2],[0.0, -self.focal, self.image_size_y/2],[0.0, 0.0, 1.0]])
    
    def destroy_actors(self):
        print('Destroying actors.')
        actors = self.world.get_actors()
        for actor in actors:
            actor.destroy()
        for actor in self.actors:
            if actor.is_alive:
                actor.destroy()
        for actor in self.vehicles_list:
            if actor.is_alive:
                actor.destroy()
        for actor in self.sensors_list:
            if actor.is_alive:
                actor.destroy()

        print("Destroyed all actors")


    def run_carla_client(self):
        client = carla.Client(self.host, self.port)
        client.set_timeout(20.0)
        self.available_maps = client.get_available_maps()
        print("Available maps are: ", self.available_maps)

        logging.info('listening to server %s:%s', self.host, self.port)

        for episode in range(self.number_of_episodes):
            print("Starting episode number %d" %episode)
                        
            uuid_run = str(uuid.uuid1())

            episode_path = os.path.join(self.base_path, "episode_{}__{}".format(str(episode), uuid_run))
            os.mkdir(episode_path)
            np.save(os.path.join(episode_path,"intrinsics.npy"), self.K)
            
            cur_map = random.choice(self.available_maps)
            print("About to load the map %s" %cur_map)

            self.world = client.load_world(cur_map)
            
            self.world.tick()

            # Initialize the actor lists
            self.vehicles_list = []
            self.sensors_list = []
            self.vehicle_dir_paths = []
            self.actors = []
            self.position_list = []
            self.rotation_list = []
            self.camR_sensor_indices = []
            self.vehicle_bbox_list = []
            
            # Get all the blueprints
            blueprints = self.world.get_blueprint_library().filter(self.filterv)

            for blueprint in blueprints:
                print(blueprint.id)
                for attr in blueprint:
                    print('  - {}'.format(attr))
            

            # Get all the spawn points
            spawn_points = self.world.get_map().get_spawn_points()
            random.shuffle(spawn_points)

            self.spawn_vehicles(blueprints, episode_path, spawn_points)
            # self.spawn_points = spawn_points
            print("CamR sensor indices are : ", self.camR_sensor_indices)

            print("Done with actor creation")
            print("Total number of sensors are: ", len(self.sensors_list))
        
            self.world.tick()
            
            last_saved_frame = 0
            # Create a synchronous mode context.
            with CarlaSyncMode(self.world, self.sensors_list, fps=self.fps) as sync_mode:
                cnt = 0
                for v in self.vehicles_list:
                    # print("Bounding box for this vehicle is: ", v.bounding_box.location, v.bounding_box.extent)
                    bbox_loc, bbox_ext = v.bounding_box.location, v.bounding_box.extent
                    bbox = [bbox_loc.x - bbox_ext.x, bbox_loc.y - bbox_ext.y, bbox_loc.z - bbox_ext.z, bbox_loc.x + bbox_ext.x, bbox_loc.y + bbox_ext.y, bbox_loc.z + bbox_ext.z]
                    self.vehicle_bbox_list.append(bbox)
                    print("bbox coords are: ", bbox)
                    v.set_autopilot(True)
                    
                print("All vehicles put to autopilot")
                self.world.tick()
                while True:

                    if cnt == self.num_datapoints_per_episode:
                        print("Done with episode %d." %episode)
                        time.sleep(3)
                        # self.destroy_actors()
                        break

                    # Randomly change weather
                    weather = carla.WeatherParameters(
                        cloudiness=np.random.randint(0, 70),
                        precipitation=np.random.randint(0, 75),
                        sun_altitude_angle=np.random.randint(30, 90))
                    self.world.set_weather(weather)
                    self.world.tick()
                    # print("Randomly set weather")

                    # Randomly teleport vehicle
                    # random.shuffle(spawn_points)
                    # self.vehicles_list[0].set_transform(spawn_points[0])
                    # self.world.tick()
                    # print("Randomly teleported vehicle")

                    # print("Getting the data")
                    # Advance the simulation and wait for the data.
                    data, frame = sync_mode.tick(timeout=12.0)
                    # print('Got the data :))')
                    data = data[1:] # Remove the world tick datapoint
                    # print("Location of car is: ", self.vehicles_list[-1].get_location())
                    if frame-last_saved_frame > self.min_frame_difference: # Don't want too similar frames
                        print("Data looks different. Will save")
                        cnt += 1
                        last_saved_frame = frame
                        self.save_data(data, frame)
                        
                        if self.should_randomize_camR:
                            self.randomize_camR()
                    # else:
                    #     print("Frame is too recent. Skipping")

    
    def randomize_camR(self):
        print("Randomizing the camRs")
        i=0
        while True:
            if i>=len(self.camR_sensor_indices):
                break
            idx = self.camR_sensor_indices[i]
            actual_rotation = self.rotation_list[idx]
            actual_position = self.position_list[idx]
            print("The actual camR position is: ", actual_position)
            print("The actual camR rotation is : ", actual_rotation)
            rand_x, rand_y, rand_z = np.random.random_sample((3))
            
            rand_x = self.camR_Xrange[0] + (self.camR_Xrange[1] - self.camR_Xrange[0])*rand_x
            rand_y = self.camR_Yrange[0] + (self.camR_Yrange[1] - self.camR_Yrange[0])*rand_y
            rand_z = self.camR_Zrange[0] + (self.camR_Zrange[1] - self.camR_Zrange[0])*rand_z

            transform = carla.Transform(carla.Location(x=rand_x, y=rand_y, z=rand_z), carla.Rotation(pitch=actual_rotation[0], yaw=actual_rotation[1], roll=actual_rotation[2]))

            for j in range(i, i+self.num_types_of_sensors):
                idx = self.camR_sensor_indices[j]
                self.position_list[idx] = [rand_x, rand_y, rand_z]
                # https://github.com/carla-simulator/carla/issues/857
                self.sensors_list[idx].set_transform(transform)
            
            i += self.num_types_of_sensors


    def save_data(self, data, framenum):
        idx = 0
        for v in range(self.num_vehicles):
            vehicle_path = self.vehicle_dir_paths[v]
            
            d = {}
            d["num_camRs"] = self.num_camRs
            d["bounding_box"] = self.vehicle_bbox_list[v]
            for j in self.sensor_names:
                d["{}_data".format(carla_utils.get_sensor_name(j))] = []
                d["{}_extrinsics".format(carla_utils.get_sensor_name(j))] = []
                d["{}_cam_to_car_transform_locs".format(carla_utils.get_sensor_name(j))] = []
                d["{}_cam_to_car_transform_rots".format(carla_utils.get_sensor_name(j))] = []

            for i in range(self.num_locations_per_vehicle):
                for j in self.sensor_names:
                    data_instance = data[idx]
                    if "rgb" in j:
                        processed_data = image_converter.to_rgb_array(data_instance)
                    elif "depth" in j:
                        processed_data = image_converter.depth_in_meters(data_instance)
                    elif "semantic" in j:
                        # TODO: handle R channel properly here.
                        processed_data = image_converter.to_rgb_array(data_instance)
                    else:
                        print("Invalid sensor type %s. Quitting" %j)
                        exit(1)

                    # print("The transform for this camera is: ", self.sensors_list[idx].get_transform())
                    # transform = carla.Transform(carla.Location(x=20.5, y=0, z=0))
                    # https://github.com/carla-simulator/carla/issues/857
                    # self.sensors_list[idx].set_transform(transform)
                    # self.world.tick()
                    # print("The new transform for this camera is: ", self.sensors_list[idx].get_transform())
                    # print("The transform for the vehicle is: ", self.vehicles_list[v].get_transform())
                    d["{}_data".format(carla_utils.get_sensor_name(j))].append(processed_data)
                    d["{}_extrinsics".format(carla_utils.get_sensor_name(j))].append(carla_utils.get_extrinsics_for_data(data_instance))

                    d["{}_cam_to_car_transform_locs".format(carla_utils.get_sensor_name(j))].append(self.position_list[idx])
                    d["{}_cam_to_car_transform_rots".format(carla_utils.get_sensor_name(j))].append(self.rotation_list[idx])
                    idx += 1
            
            for j in self.sensor_names:
                d["{}_data".format(carla_utils.get_sensor_name(j))] = np.stack(d["{}_data".format(carla_utils.get_sensor_name(j))])
                d["{}_extrinsics".format(carla_utils.get_sensor_name(j))] = np.stack(d["{}_extrinsics".format(carla_utils.get_sensor_name(j))])
                d["{}_cam_to_car_transform_locs".format(carla_utils.get_sensor_name(j))] = np.stack(d["{}_cam_to_car_transform_locs".format(carla_utils.get_sensor_name(j))])
                d["{}_cam_to_car_transform_rots".format(carla_utils.get_sensor_name(j))] = np.stack(d["{}_cam_to_car_transform_rots".format(carla_utils.get_sensor_name(j))])

            pickle_fname = "vehicle-{}_frame-{}.p".format(v, framenum)
            print("Saving data in pickle file: %s" %pickle_fname)
            with open(os.path.join(vehicle_path, pickle_fname), 'wb') as f:
                pickle.dump(d, f)
            
    '''
    Spawns the vehicles and attaches sensors to it.
    '''
    def spawn_vehicles(self, blueprints, episode_path, spawn_points):
        number_of_spawn_points = len(spawn_points)
        num_vehicles = min(self.num_vehicles, number_of_spawn_points)

        for n, transform in enumerate(spawn_points):
            if n >= num_vehicles:
                break
            
            # Set recommended attributes and spawn the vehicle.
            blueprint = blueprints[self.vehicle_blueprint_number]
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            if blueprint.has_attribute('driver_id'):
                driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                blueprint.set_attribute('driver_id', driver_id)

            blueprint.set_attribute('role_name', 'autopilot')
            vehicle = self.world.try_spawn_actor(blueprint, transform)
            self.actors.append(vehicle)

            vehicle_path = os.path.join(episode_path, str(blueprint.id))
            os.mkdir(vehicle_path)
            self.vehicle_dir_paths.append(vehicle_path)
            print("Created the directory for vehicle: ", vehicle.id)
            
            # Attach all the sensors to the vehicle
            is_camR_pos = True
            for position, rotation in zip(self.positions, self.rotations):
                for sensor in self.sensor_names:
                    # print("Attaching pos and rot")
                    vehicle_sensor = self.get_sensor(vehicle, position, rotation, sensor)
                    self.actors.append(vehicle_sensor)
                    self.sensors_list.append(vehicle_sensor)
                    if is_camR_pos:
                        self.camR_sensor_indices.append(len(self.sensors_list)-1)
                is_camR_pos = False
            
            print("Successfully attached all the sensors to vehicle: ", vehicle.id)
                
            self.vehicles_list.append(vehicle)
    
    
    def get_sensor(self, vehicle, position, rotation, sensor):
        blueprint = self.world.get_blueprint_library().find(sensor)

        blueprint.set_attribute('image_size_x', str(self.image_size_x))
        blueprint.set_attribute('image_size_y', str(self.image_size_y))
        blueprint.set_attribute('fov', str(self.fov))
        blueprint.set_attribute('role_name', str(vehicle.id))
         
        # Set the time in seconds between sensor captures
        blueprint.set_attribute('sensor_tick', '0.0')
        
        # Provide the position of the sensor relative to the vehicle.
        transform = carla.Transform(carla.Location(x=position[0], y=position[1], z=position[2]), carla.Rotation(pitch=rotation[0], yaw=rotation[1], roll=rotation[2]))
        
        self.position_list.append(position)
        self.rotation_list.append(rotation)
        # Tell the world to spawn the sensor, don't forget to attach it to your vehicle actor.
        sensor = self.world.spawn_actor(blueprint, transform, attach_to=vehicle)
        
        return sensor


    def initiate(self):
        if not os.path.exists(self.base_path):
            os.mkdir(self.base_path)
        
        while True:
            # try:
            self.run_carla_client()
            print('Done.')
            return

            # except Exception as error:
            #     print("Error occured while interacting with carla.")
            #     print(error)
            #     logging.error(error)
            #     time.sleep(1)

if __name__ == '__main__':
    vehicle_num = int(sys.argv[1])
    print("Vehicle num received is: ", vehicle_num)
    cmr = CarlaMultiviewRunner(vehicle_num)
    cmr.initiate()
