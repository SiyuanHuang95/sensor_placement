from generator_sensors import CSensorFactory
from fence import CFence
from human import CHuman
from robot import CRobot
from lidar import CLidar
from mat import CMate
from utils import *

import matplotlib.pyplot as plt
import numpy as np


sensors = []
for sensor_name in CSensorFactory.sensor_name_gene(5):
    parameters_ = CSensorFactory.sensor_dict(sensor_name)
    sensors.append(CSensorFactory.create_sensor(sensor_name, parameters_))

human = CHuman('Worker1', start_vel=1, start_pos_x=-2.5, start_pos_y=-3, heading=np.pi/100)
robot = CRobot('Robot1', start_vel=0.1, start_pos=0)

fig, axes = plt.subplots(1,1)
plt.ion()
current_time = 0
dt = 0.010 # time step
simulation_time = 10

while current_time < simulation_time:
    human.update(dt, omega=0.1)
    robot.update()
    axes.cla()
    # axes.axis("equal")
    axes = plt.gca()
    axes.set_xlim([-10, 10])
    axes.set_ylim([-10, 10])
    human.plot(axes)
    robot.plot(axes)
    draw_warn_zone(axes, robot)
    for sensor in sensors:
        sensor.plot(axes)
        sensor.detection(human, axes)
    plt.pause(0.01)
    plt.show()
