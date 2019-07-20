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
    area_length = 10

    @staticmethod
    def create_sensor(name, parameters):
        if name not in CSensorFactory.factories:
            CSensorFactory.factories[name] = eval(name + '.Factory()')
        return CSensorFactory.factories[name].create(parameters)

    @staticmethod
    def sensor_name_gene(n):
        types = CSensor.__subclasses__()
        for i in range(10*n):
            yield random.choice(types).__name__

    @staticmethod
    def get_position():
        position = random.randint(-CSensorFactory.area_length, CSensorFactory.area_length)
        while abs(position) < 0.5:
            position = random.randint(-CSensorFactory.area_length, CSensorFactory.area_length)
        return position

    @staticmethod
    def sensor_dict(sensor_name):
        if sensor_name == 'CMate':
            parameters = {'x': CSensorFactory.get_position(), 'y': CSensorFactory.get_position(), 'width': random.randint(3, 5),
                          'length': random.randint(4, 6)}
        elif sensor_name == 'CLidar':
            parameters = {'x': CSensorFactory.get_position(), 'y': CSensorFactory.get_position(), 'lidar_range': random.randint(3, 10),
                          'angle_res': random.uniform(0.01, 0.1), 'max_angle': 3.14 * random.random(),
                          'rate': random.randint(10, 100)}
        elif sensor_name == 'CFence':
            parameters = {'x': CSensorFactory.get_position(), 'y': CSensorFactory.get_position(), 'length': random.randint(3, 5)}
        return parameters
