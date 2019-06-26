from __future__ import generators
from sensor import CSensor
import random
from fence import CFence
from human import CHuman
from robot import CRobot
from lidar import CLidar
from mat import CMate


class CSensorFactory:
    factories = {}

    @staticmethod
    def create_sensor(name, parameters):
        if name not in CSensorFactory.factories:
            CSensorFactory.factories[name] = eval(name + '.Factory()')
        return CSensorFactory.factories[name].create(parameters)

    @staticmethod
    def sensor_name_gene(n):
        types = CSensor.__subclasses__()
        for i in range(n):
            yield random.choice(types).__name__

    @staticmethod
    def sensor_dict(sensor_name):
        if sensor_name == 'CMate':
            parameters = {'x': random.randint(-10, 10), 'y': random.randint(-10, 10), 'width': random.randint(3, 5),
                          'length': random.randint(4, 6)}
        elif sensor_name == 'CLidar':
            parameters = {'x': random.randint(-10, 10), 'y': random.randint(-10, 10), 'lidar_range': random.randint(3, 10),
                          'angle_res': random.uniform(0.01, 0.1), 'max_angle': 3.14 * random.random(),
                          'rate': random.randint(10, 100)}
        elif sensor_name == 'CFence':
            parameters = {'x': random.randint(-10, 10), 'y': random.randint(-10, 10), 'length': random.randint(3, 5)}
        return parameters
