from __future__ import generators
import random
from sensor import CSensor
from fence import CFence
from human import CHuman
from robot import CRobot
from lidar import CLidar
from mat import CMate


class CSensorFactory:
    factories = {}

    @staticmethod
    def create_sensor(name, parameters=[]):
        if name not in CSensorFactory.factories:
            CSensorFactory.factories[name] = eval(name + '.Factory()')
        return CSensorFactory.factories[name].create(parameters)


def sensor_name_gene(n):
    types = CSensor.__subclasses__()
    for i in range(n):
        yield random.choice(types).__name__


sensors = [CSensorFactory.create_sensor(i) for i in sensor_name_gene(5)]
for sensor in sensors:
    print(sensor)

