from __future__ import generators

import random
from sensor_class.sensor import CSensor
from sensor_class.fence import CFence
from sensor_class.lidar import CLidar
from sensor_class.mat import CMate
import numpy as np


class CSensorFactory:
    factories = {}
    area_length = 10
    random_generate = False
    dangerous_zone_radius = 1.5

    @staticmethod
    def create_sensor(name, parameters):
        if name not in CSensorFactory.factories:
            # here we use a dictionary to store the API for the factory
            # key is the sensor name, value is the sensor.Factory(), e.g. the member method of the corresponding class
            CSensorFactory.factories[name] = eval(name + '.Factory()')
        return CSensorFactory.factories[name].create(parameters)

    # we choose the sensor type first based on the random choice of the sensor names
    @staticmethod
    def sensor_name_gene(n):
        name_list = []
        types = CSensor.__subclasses__()
        # return the sensor names which coming from the subclass of the same base class
        for i in range(10 * n):
            name_list.append(random.choice(types).__name__)
        return name_list

    # dummy API to get sensor positions considering the fact that one sensor should not be generated two away from the
    # robot, otherwise it will be totally a waste
    @staticmethod
    def get_position_lidar():
        position = random.randint(-7, 7)
        while abs(position) < 0.2:
            position = CSensorFactory.get_position_lidar()
        return position

    @staticmethod
    def get_position_mate():
        position = random.randint(-CSensorFactory.dangerous_zone_radius - 2, CSensorFactory.dangerous_zone_radius + 2)
        while abs(position) < 0.2:
            position = CSensorFactory.get_position_mate()
        return position

    @staticmethod
    def get_position_fence():
        position = random.randint(-CSensorFactory.dangerous_zone_radius - 2, CSensorFactory.dangerous_zone_radius + 2)
        while abs(position) < CSensorFactory.dangerous_zone_radius:
            position = CSensorFactory.get_position_fence()
        return position

    # the sensor dictionary stores some parameters which is more reasonable than randomly generated, the parameters here
    # could come from the marketing
    @staticmethod
    def sensor_dict(sensor_name, random_generate=False):
        if random_generate:
            if sensor_name == 'CMate':
                parameters = {'x': CSensorFactory.get_position_mate(), 'y': CSensorFactory.get_position_mate(),
                              'width': random.randint(3, 5),
                              'length': random.randint(4, 6)}
            elif sensor_name == 'CLidar':
                parameters = {'x': CSensorFactory.get_position_lidar(), 'y': CSensorFactory.get_position_lidar(),
                              'lidar_range': random.randint(3, 8),
                              'angle_res': random.uniform(0.01, 0.1), 'max_angle': 6.28 * random.uniform(0.1, 1),
                              'rate': random.randint(10, 100)}
            elif sensor_name == 'CFence':
                parameters = {'x': CSensorFactory.get_position_fence(), 'y': CSensorFactory.get_position_fence(),
                              'length': random.randint(3, 5)}
        else:

            if sensor_name == 'CMate':
                mate_width_list = [0.5, 0.7, 0.75, 1.5]
                mate_length_list = [1.0, 0.75, 1.5]

                parameters = {'x': CSensorFactory.get_position_mate(), 'y': CSensorFactory.get_position_mate(),
                              'width': random.choice(mate_width_list), 'length': random.choice(mate_length_list)}
            elif sensor_name == 'CLidar':

                lidar_range_list = [5, 6.5, 7.0]
                lidar_angle_list = [45, 70, 100, 120, 150, 190]

                parameters = {'x': CSensorFactory.get_position_lidar(), 'y': CSensorFactory.get_position_lidar(),
                              'lidar_range': random.choice(lidar_range_list),
                              'angle_res': random.uniform(0.01, 0.1),
                              'max_angle': random.choice(lidar_angle_list) / 180 * 3.1415926,
                              'rate': random.randint(10, 100)}
            elif sensor_name == 'CFence':

                fence_width_list = [0.1, 0.5]
                fence_length_list = [1.0, 1.5, 2.0]

                parameters = {'x': CSensorFactory.get_position_fence(), 'y': CSensorFactory.get_position_fence(),
                              'length': random.choice(fence_length_list), 'width': random.choice(fence_width_list)}
        return parameters
