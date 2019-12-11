from sensor_class.sensor import CSensor
import numpy as np
import math
import matplotlib.pyplot as plt
from sensor_class.utils import *

dt = 0.01
robot_base_radius = 0.


class CLidar(CSensor):
    lidar_number = 0

    def __init__(self, name='lidar', x=0, y=0, lidar_range=7, angle_res=0.01, max_angle=math.pi / 2, rate=20):
        super(CLidar, self).__init__(name, x, y)
        self.range = lidar_range
        self.angle_res = angle_res
        self.max_angle = max_angle
        self.range_noise = 0.01
        self.heading = math.atan2(self.y_base, self.x_base)
        self.rate = rate
        self.time = 0
        self.lidar_scanner = 0
        self.distance2robot = np.hypot(self.x_base, self.y_base)
        self.occlusion_angle = np.arcsin(robot_base_radius / self.distance2robot)
        self.status = 0
        CLidar.lidar_number += 1

    def detection(self, human, plt):
        self.time += dt
        lidar_time_gap = 1 / self.rate
        if self.time // lidar_time_gap - self.lidar_scanner == 1:
            rx, ry = self.object_detection(human)
            self.lidar_scanner += 1
            if len(rx) > 1:
                self.plot_scan(plt, rx, ry, color='g')
                self.signal_output(rx, ry)

    # object detection main function
    def object_detection(self, human):
        x, y, angle, r = [], [], [], []
        gx, gy = human.standing_area()
        for vx, vy in zip(gx, gy):
            vx = vx - self.x_base
            vy = vy - self.y_base
            vangle = math.atan2(vy, vx)
            vr = np.hypot(vx, vy)
            x.append(vx)
            y.append(vy)
            angle.append(vangle)
            r.append(vr)
        distance = np.hypot(np.hypot(self.x_base - human.x, self.y_base - human.y), human.arm)
        rx, ry = self.ray_casting_filter(x, y, angle, r, distance)
        rx = [x + self.x_base for x in rx]
        ry = [y + self.y_base for y in ry]
        return rx, ry

    def cover_area(self):
        return self.max_angle / (2 * np.pi) * np.pi * np.square(self.range)

    # for visualization when generating the sensors
    def cover_bins(self):
        heading = np.pi - math.atan2(self.x_base, self.y_base)
        x_ = [self.range * np.sin(theta) * np.cos(heading) - self.range * np.cos(theta) *
              np.sin(heading) + self.x_base
              for theta in np.arange(-self.max_angle / 2, self.max_angle / 2, step=0.2)]

        y_ = [self.range * np.sin(theta) * np.sin(heading) + self.range * np.cos(theta) *
              np.cos(heading) + self.y_base
              for theta in np.arange(-self.max_angle / 2, self.max_angle / 2, step=0.2)]
        return x_, y_

    # give a rough status judgement
    def signal_output(self, rx, ry, safe=3, dangerous_=2):
        if len(rx) > 0:
            distance_min = np.min([math.hypot(ix, iy) for (ix, iy) in zip(rx, ry)])
            if distance_min < dangerous_:
                self.status = 2
            elif distance_min < safe:
                self.status = 1
            else:
                self.status = 0

    @staticmethod
    def unit_vector(vector):
        return vector / np.linalg.norm(vector)

    @staticmethod
    def angle_between(v1, v2):
        v1_u = CLidar.unit_vector(v1)
        v2_u = CLidar.unit_vector(v2)
        return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

    # ray casting filter
    def ray_casting_filter(self, xl, yl, thetal, rangel, distance):
        rx, ry = [], []
        # init the lidar results with "inf"
        # at first, we assume it has cover range of (2*pi), just for one easy calculation
        rangedb = [float("inf") for _ in range(
            int(np.floor((np.pi * 2.0) / self.angle_res)) + 1)]

        for i in range(len(thetal)):
            angleid = int(round(thetal[i] / self.angle_res))

            vector_1 = (xl[i], yl[i])
            vector_2 = (-self.x_base, -self.y_base)
            temp_angle = CLidar.angle_between(vector_1, vector_2)
            angle_in_range = (np.abs(temp_angle) < self.max_angle / 2)
            angle_in_occlusion = np.abs(temp_angle) < self.occlusion_angle
            robot_in_between = distance < self.distance2robot
            if robot_in_between:
                # when robot is between human and laser, we should consider the occlusion of the laser
                # bins with robot base
                if rangedb[angleid] > rangel[i] and rangel[i] < self.range and rangel[i] < distance and angle_in_range:
                    rangedb[angleid] = rangel[i]
            elif not robot_in_between:
                if rangedb[angleid] > rangel[i] and rangel[i] < self.range and rangel[i] < distance and angle_in_range \
                        and not angle_in_occlusion:
                    rangedb[angleid] = rangel[i]

        for i in range(len(rangedb)):
            t = i * self.angle_res
            if rangedb[i] != float("inf"):
                rx.append(rangedb[i] * np.cos(t))
                ry.append(rangedb[i] * np.sin(t))
        return rx, ry

    def plot(self, fig):
        fig.plot(self.x_base, self.y_base, "^g")
        plt.text(self.x_base, self.y_base, self.name)
        if self.status == 0:
            plt.text(-7, 7, "Safety", fontsize=14, color='green')
        elif self.status == 1:
            plt.text(-7, 7, "Critical", fontsize=14, color='yellow')
        else:
            plt.text(-7, 7, "Dangerous", fontsize=14, color='red')

    # used for visualization when generate the sensors
    def visualization(self, fig):
        x, y = self.cover_bins()
        temp_x, temp_y = x[0], y[0]
        for (ix, iy) in zip(x, y):
            plt.plot([self.x_base, ix], [self.y_base, iy], 'b')
            plt.plot([temp_x, ix], [temp_y, iy], 'b')
            temp_x, temp_y = ix, iy
        plt.text(-7, -7, f"Generate One LiDAR", fontsize=14, color='red')

    # visualize the detection results
    def plot_scan(self, plt, ox, oy, color='g'):
        x = [ox[i] for i in range(len(ox))]
        y = [oy[i] for i in range(len(ox))]

        for (ix, iy) in zip(x, y):
            plt.plot([self.x_base, ix], [self.y_base, iy], color)

    # check how many grid points of the dangerous zoe are covered
    def coverage_dangerous_zone(self, coverage_dict, dangerous_zone_radius=1.5):
        keys = list(coverage_dict.keys())
        bad_placement_flag = True
        if np.hypot(self.x_base, self.y_base) < (self.range + dangerous_zone_radius):
            for i in range(len(keys)):
                vector_1 = (keys[i][0], keys[i][1])
                vector_2 = (-self.x_base, -self.y_base)

                temp_angle = np.abs(CLidar.angle_between(vector_1, vector_2))
                angle_in_range = (temp_angle < self.max_angle / 2)
                angle_in_occlusion = temp_angle < self.occlusion_angle

                temp_distance = np.hypot(keys[i][0] - self.x_base, keys[i][1] - self.y_base)
                distance_in_range = (temp_distance <= self.range)
                if temp_distance < self.distance2robot:
                    if angle_in_range and distance_in_range:
                        coverage_dict[(keys[i][0], keys[i][1])] += 1
                        bad_placement_flag = False
                else:
                    if angle_in_range and distance_in_range and not angle_in_occlusion:
                        coverage_dict[(keys[i][0], keys[i][1])] += 1
                        bad_placement_flag = False

            return bad_placement_flag
        else:
            return bad_placement_flag

    # the API for sensor factory
    class Factory:
        @staticmethod
        def create(parameters): return CLidar(**parameters)
