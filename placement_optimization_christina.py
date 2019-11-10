from sensor_factory.generator_sensors import CSensorFactory
from sensor_factory.space_control import CSpace_control
from sensor_class.human import CHuman
from sensor_class.robot import CRobot
from sensor_class.utils import *
from sympy import Point, Polygon, pi
from numpy.linalg import norm
from matplotlib.patches import Wedge
from shapely.geometry import asPolygon, LineString
from shapely.geometry import Polygon

import matplotlib.pyplot as plt
import numpy as np
import math
import random


# def cost_func(position_sensor):
#    cover_area = sensor.cover_area()
#    #sensor_size depending on type -->
#    #mat: = cover_area
#    #fence: = cover_area (you aren't able to approach region behind fence)
#    #lidar: = math.pi * r**2
#    #cost: depending on type
#    #
#    return 1


# definition of global safety function, every coordinate is tested whether it's covered by sensor (doesn't work yet,
# problems with "coord.within(poly)"-function). The plan was to make this function
# a second constraint for producing new sensors.
def global_safety_func(position):
    dx = 0.05
    dy = 0.05
    cover_bool = 0
    for i in np.arange(start=-5, stop=5, step=dx):
        for j in np.arange(start=-5, stop=5, step=dy):
            coord = [i, j]
            cover_bool = 0
            for k in range(len(position)):
                poly = create_polygon(position[k])
            # print(coord.within(poly))
            if cover_bool == 1:
                return 1
            else:
                return 0


# returns sensor type as string
def sensor_type(position_sensor):
    s = str(position_sensor)
    substring = ","
    type = s.count(substring)
    if type == 2:
        return "fence"
    if type == 3:
        return "mat"
    if type == 5:
        return "lidar"


# creates an perpendicular vector for some geometrical calculations in polygon creation
def perpendicular_vector(vec):
    # x = y = z = 0 is not an acceptable solution
    if vec[0] == vec[1] == 0:
        raise ValueError('zero-vector')
    elif vec[0] == 0:
        return [1,0]
    elif vec[1] == 0:
        return [0,1]
    elif vec[0] != 0 and vec[1] != 0:
        a = 1
        b = -(vec[0] * a)/vec[1]
        abs = np.sqrt(a*a + b*b)
        a = a/abs
        b = b/abs
        return np.array([a,b])


# creates a polygon out of position and parameters of sensor
def create_polygon(position_sensor):
    s = str(position_sensor)
    x = float(s.split("{'x': ")[1].split(", 'y':")[0])
    y = float(s.split("'y': ")[1].split(", '")[0])
    theta = math.atan2(y,x)
    if sensor_type(s) == "fence":
        length = float(s.split("'length': ")[1].split("}")[0])
        width = 0.5
        center = np.array([x,y])
        center_norm = norm(center)
        # print("normed center: " + str(center_norm))
        # print("perpendicular = " + str(perpendicular_vector(center)))
        # print("center = " + str(center))
        a = center + (center/center_norm) * width/2 + perpendicular_vector(center) * length/2
        # print("a = " + str(a))
        b = center + (center/center_norm) * width/2 - perpendicular_vector(center) * length/2
        # print("b = " + str(b))
        c = center - (center/center_norm) * width/2 - perpendicular_vector(center) * length/2
        # print("c = " + str(c))
        d = center - (center/center_norm) * width/2 + perpendicular_vector(center) * length/2
        # print("d = " + str(d))
        poly = Polygon([a, b, c, d])
        return poly
    if sensor_type(s) == "mat":
        width = float(s.split("'width': ")[1].split(", 'length':")[0])
        length = float(s.split("'length': ")[1].split("}")[0])
        center = np.array([x,y])
        center_norm = norm(center)
        a = center + (center/center_norm) * width/2 + perpendicular_vector(center) * length/2
        b = center - (center/center_norm) * width/2 + perpendicular_vector(center) * length/2
        c = center - (center/center_norm) * width/2 - perpendicular_vector(center) * length/2
        d = center + (center/center_norm) * width/2 - perpendicular_vector(center) * length/2
        poly = Polygon([a,b,c,d])
        return poly
    if sensor_type(s) == "lidar":
        r = float(s.split("'lidar_range': ")[1].split(", 'angle_res':")[0])
        angle = float(s.split("'max_angle': ")[1].split(", 'rate':")[0])
        center = np.array([x,y])
        theta1 = theta - angle/2
        theta2 = theta + angle/2
        wedge_patch = Wedge(center,r,theta1, theta2)
        wedge_path = wedge_patch.get_path()
        wedge_polygon = asPolygon(wedge_path.vertices).buffer(0)
        return wedge_polygon


# calculates the total overlap of newly generated sensor with existing ones
def sensor_overlap(position_sensor,parameter_new):
    overlap = 0
    for i in range(len(position_sensor)):
        new_sensor = create_polygon(parameter_new)
        sensor_i = create_polygon(position_sensor[i])
        intersection = new_sensor.intersection(sensor_i)
        print("intersection area = " + str(intersection.area))
        overlap += float(intersection.area)
        i += 1
    return overlap

sensor_number = 0
sensors = []
position = []
cover_area = 0
area = 30*30
i = 0


# ___________________actual code:__________________________________
# sensors are generated until cover_area >= area of square around robot (next step would be
# to change that into fullfilled global safety function).
# Generation is only completed if overlap with existing sensors is less than 5%.
while cover_area < area:
    parameters = CSensorFactory()
    generate = CSensorFactory()
    for sensor_name in CSensorFactory.sensor_name_gene(1):
        parameters_ = CSensorFactory.sensor_dict(sensor_name)
        generate_ = CSensorFactory.create_sensor(sensor_name, parameters_)
        intersection = sensor_overlap(position,parameters_)
        area_new_sensor = generate_.cover_area()
        percentage_overlap = intersection/area_new_sensor
        print("intersection =" + str(intersection))
        print("area_new_sensor = " + str(area_new_sensor))
        print("percentage overlap = " + str(intersection/area_new_sensor))
        if CSpace_control.check(generate_) and percentage_overlap < 0.05:
            sensor_number += 1
            sensors.append(generate_)
            position.append(parameters_)
    for sensor in sensors:
        cover_area += sensor.cover_area()


print("number sensors:" + str(len(position)))
print("cover area: " + str(cover_area))
# print("global safety function = " + str(global_safety_func(position)))

robot = CRobot('Robot1', start_vel=0.1, start_pos=0)

fig, axes = plt.subplots(1,1)


axes.cla()
axes.axis("equal")
axes = plt.gca()
axes.set_xlim([-10, 10])
axes.set_ylim([-10, 10])
robot.plot(axes)
draw_warn_zone(axes, robot)
for sensor in sensors:
    sensor.plot(axes)
plt.show()
