
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

try:
    import queue
except ImportError:
    import Queue as queue

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla


class CarlaSyncMode(object):
    """
    Context manager to synchronize output from different sensors. Synchronous
    mode is enabled as long as we are inside this context

        with CarlaSyncMode(world, sensors) as sync_mode:
            while True:
                data = sync_mode.tick(timeout=1.0)

    """

    def __init__(self, world, rgb_sensors, **kwargs):
        self.world = world
        self.sensors = rgb_sensors
        self.frame = None
        self.delta_seconds = 1.0 / kwargs.get('fps')
        self._queues = []
        self._settings = None

    def __enter__(self):
        self._settings = self.world.get_settings()
        self.frame = self.world.apply_settings(carla.WorldSettings(
            no_rendering_mode=False,
            synchronous_mode=True,
            fixed_delta_seconds=self.delta_seconds))

        def make_queue(register_event):
            q = queue.Queue()
            register_event(q.put)
            self._queues.append(q)

        make_queue(self.world.on_tick)
        for sensor in self.sensors:
            make_queue(sensor.listen)
        return self

    def tick(self, timeout):
        self.frame = self.world.tick()
        # print("world tick done")
        # print("self frame is: ", self.frame)
        data = []
        for q in self._queues:
            # print("running for queue: ", q)
            data_i = self._retrieve_data(q, timeout)
            data.append(data_i)
        # data = [self._retrieve_data(q, timeout) for q in self._queues]
        # print("Got data from all queues with frames:")
        # print(self.frame)
        # for x in data:
        #     print(x.frame)
        assert all(x.frame == self.frame for x in data)
        
        return data, self.frame

    def __exit__(self, *args, **kwargs):
        self.world.apply_settings(self._settings)

    def _retrieve_data(self, sensor_queue, timeout):
        while True:
            data = sensor_queue.get(timeout=timeout)
            # print("Got data from the queue with frame: ", data.frame)
            if data.frame == self.frame:
                return data
