from sensor import CSensor
import numpy as np
import math
import matplotlib.pyplot as plt


class CLidar(CSensor):
    # Adding the Lidar frame update frequence as wa parameter
    lidar_number = 0

    def __init__(self, name='lidar', x=0, y=0, lidar_range=5, angle_res=0.01, max_angle=math.pi/2, rate=4):
        super(CLidar, self).__init__(name, x, y)
        self.range = lidar_range
        self.angle_res = angle_res
        self.max_angle = max_angle
        self.range_noise = 0.01
        self.heading = math.pi / 2 + math.atan2(self.x_base, self.y_base)
        self.rate = rate
        CLidar.lidar_number += 1

    def object_detection(self, human):
        # Note: In the first version, Lidar only detects the human
        x, y, angle, r = [], [], [], []
        gx, gy = human.standing_area()
        for vx, vy in zip(gx, gy):
            vx = vx - self.x_base
            vy = vy - self.y_base
            vangle = math.atan2(vy, vx)
            vr = np.hypot(vx, vy)
            # Should we use Gaussian to simulate the noise?
            # vr = np.hypot(vx,vy) + * random.uniform(1.0 -self.range_noise, 1.0 + self.range_noise)
            x.append(vx)
            y.append(vy)
            angle.append(vangle)
            r.append(vr)
        distance = np.hypot(np.hypot(self.x_base - human.x, self.y_base - human.y), human.arm)
        rx, ry = self.ray_casting_filter(x, y, angle, r, distance)
        rx = [x + self.x_base for x in rx]
        ry = [y + self.y_base for y in ry]
        return rx, ry

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
            angle_in_range = np.abs(CLidar.angle_between(vector_1, vector_2)) < self.max_angle

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
        plt.text(self.x_base, self.y_base, self.name)

    class Factory:
        def create(self, parameters): return CLidar(**parameters)

class CLidarPlotter:
    def __init__(self, x, y, colorcode='r'):
        self.x = x
        self.y = y
        self.color = colorcode

    def plot_scan(self, plt, ox, oy):
        x = [ox[i] for i in range(len(ox))]
        y = [oy[i] for i in range(len(ox))]

        for (ix, iy) in zip(x, y):
            plt.plot([self.x, ix], [self.y, iy], self.color)