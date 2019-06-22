from fence import CFence
from human import CHuman
from robot import CRobot
from lidar import CLidar, CLidarPlotter
from mat import CMate, CMatePlotter
from utils import *

import matplotlib.pyplot as plt
import numpy as np


mate = CMate('Mate1', x=-3, y=-4, width=4, length=6)
fence = CFence('Fence1', x=4, y=-3, width=0.5, length=4)
lidar = CLidar(x=2, y=2)
robot = CRobot('Robot1', start_vel=0.1, start_pos =0)
human = CHuman('Worker1', start_vel=1, start_pos_x=-2.5, start_pos_y=-3, heading=np.pi/100)
lidar_plt = CLidarPlotter(x=lidar.x_base, y=lidar.y_base, colorcode='g')
mate_plt = CMatePlotter(x=mate.x_base, y=mate.y_base, colorcode='y')

current_time = 0
dt = 0.010 # time step
simulation_time = 10
fig, axes = plt.subplots(1,1)
lidar_time_gap = 1 / lidar.rate
lidar_scanner = 0
plt.ion()
rx, ry = [], []
while current_time < simulation_time:
    if current_time // lidar_time_gap -lidar_scanner == 1:
        rx, ry = lidar.object_detection(human)
        lidar_scanner += 1
        lidar.signal_output(rx, ry)

    human.update(dt, omega=0)
    robot.update()
    current_time += dt
    fence.fence_contact(human)
    mx, my = mate.detect_human(human)
    axes.cla()
    # axes.axis("equal")
    axes = plt.gca()
    axes.set_xlim([-10, 10])
    axes.set_ylim([-10, 10])
    mate.plot(axes)
    fence.plot(axes)
    lidar.plot(axes)
    mate_plt.plot_scan(axes, mx, my)
    if len(rx) > 1:
        lidar_plt.plot_scan(axes, rx, ry)
    human.plot(axes)
    robot.plot(axes)
    draw_warn_zone(axes, robot)
    plt.pause(0.01)
    plt.show()

# TODO: Implement of the factory class


