from sensor_factory.generator_sensors import CSensorFactory
from sensor_factory.space_control import CSpace_control
from human import CHuman
from robot import CRobot
from utils import *

import matplotlib.pyplot as plt
import numpy as np

sensor_number = 5
sensor_counter = 0
sensors = []
cover_area = 0
for sensor_name in CSensorFactory.sensor_name_gene(sensor_number):
    parameters_ = CSensorFactory.sensor_dict(sensor_name)
    generate_ = CSensorFactory.create_sensor(sensor_name, parameters_)
    if CSpace_control.check(generate_) and sensor_counter < sensor_number:
        sensor_counter += 1
        sensors.append(generate_)

for sensor in sensors:
    print(sensor)
    cover_area += sensor.cover_area()

human = CHuman('Worker1', start_vel=5, start_pos_x=2.5, start_pos_y=3, heading=-np.pi/1.2)
robot = CRobot('Robot1', start_vel=0.1, start_pos=0)

fig, axes = plt.subplots(1,1)
plt.ion()
current_time = 0
dt = 0.010 # time step
simulation_time = 10

while current_time < simulation_time:
    current_time += dt
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
