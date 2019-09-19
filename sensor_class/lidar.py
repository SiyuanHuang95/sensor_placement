from sensor import CSensor
import numpy as np
import math
import matplotlib.pyplot as plt

dt = 0.01

class CLidar(CSensor):
    # Adding the Lidar frame update frequence as wa parameter
    lidar_number = 0

    def __init__(self, name='lidar', x=0, y=0, lidar_range=5, angle_res=0.01, max_angle=math.pi/2, rate=20):
        super(CLidar, self).__init__(name, x, y)
        self.range = lidar_range
        self.angle_res = angle_res
        self.max_angle = max_angle
        self.range_noise = 0.01
        self.heading = math.atan2(self.y_base, self.x_base)
        self.rate = rate
        self.time = 0
        self.lidar_scanner = 0
        CLidar.lidar_number += 1

    def detection(self, human, plt):
        self.time += dt
        lidar_time_gap = 1/self.rate
        if self.time // lidar_time_gap - self.lidar_scanner == 1:
            rx, ry = self.object_detection(human)
            self.lidar_scanner += 1
            if len(rx) > 1:
                self.plot_scan(plt, rx, ry, color='g')
                self.signal_output(rx, ry)

    def object_detection(self, human):
        # Note: In the first version, Lidar only detects the human
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

    def cover_bins(self):
        heading = np.pi - math.atan2(self.x_base, self.y_base)
        x_ = [self.range * np.sin(theta) * np.cos(heading) - self.range * np.cos(theta) *
              np.sin(heading) + self.x_base
              for theta in np.arange(-self.max_angle/2, self.max_angle/2, step=0.2)]

        y_ = [self.range * np.sin(theta) * np.sin(heading) + self.range * np.cos(theta) *
              np.cos(heading) + self.y_base
              for theta in np.arange(-self.max_angle/2, self.max_angle/2, step=0.2)]
        return x_, y_

    @staticmethod
    def signal_output(rx, ry, safe=3, dangerous_=2):
        if len(rx) > 0:
            distance_min = np.min([math.hypot(ix, iy) for (ix, iy) in zip(rx, ry)])
            if distance_min < dangerous_:
                print("Dangerous")
            elif distance_min < safe:
                print("Critical")
            else:
                print("Safety")

    @staticmethod
    def unit_vector(vector):
        return vector / np.linalg.norm(vector)

    @staticmethod
    def angle_between(v1, v2):
        v1_u = CLidar.unit_vector(v1)
        v2_u = CLidar.unit_vector(v2)
        return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

    def ray_casting_filter(self, xl, yl, thetal, rangel, distance):
        rx, ry = [], []
        rangedb = [float("inf") for _ in range(
            int(np.floor((np.pi * 2.0) / self.angle_res)) + 1)]
        # init the lidar results with "inf"

        for i in range(len(thetal)):
            angleid = int(round(thetal[i] / self.angle_res))

            vector_1 = (xl[i], yl[i])
            vector_2 = (-self.x_base, -self.y_base)
            angle_in_range = (np.abs(CLidar.angle_between(vector_1, vector_2)) < self.max_angle/2)

            if rangedb[angleid] > rangel[i] and rangel[i] < self.range and rangel[i] < distance and angle_in_range:
                rangedb[angleid] = rangel[i]

        for i in range(len(rangedb)):
            t = i * self.angle_res
            if rangedb[i] != float("inf"):
                rx.append(rangedb[i] * np.cos(t))
                ry.append(rangedb[i] * np.sin(t))
        return rx, ry

    def plot(self, fig):
        fig.plot(self.x_base, self.y_base, "^g")
        x, y = self.cover_bins()
        temp_x, temp_y = x[0], y[0]
        for (ix, iy) in zip(x, y):
            plt.plot([self.x_base, ix], [self.y_base, iy], 'b')
            plt.plot([temp_x, ix], [temp_y, iy], 'b')
            temp_x, temp_y = ix, iy
        plt.text(self.x_base, self.y_base, self.name)

    def plot_scan(self, plt, ox, oy, color='g'):
        x = [ox[i] for i in range(len(ox))]
        y = [oy[i] for i in range(len(ox))]

        for (ix, iy) in zip(x, y):
            plt.plot([self.x_base, ix], [self.y_base, iy], color)

    class Factory:
        @staticmethod
        def create(parameters): return CLidar(**parameters)
