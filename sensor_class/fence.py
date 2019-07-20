from sensor import CSensor
import numpy as np
import math
import matplotlib.pyplot as plt


class CFence(CSensor):  # Fence only influences the movement direction of human
    fence_number = 0

    def __init__(self, name='fence', x=0, y=0, length=1):
        super(CFence, self).__init__(name, x, y)
        self.length = length
        self.width = self.width_value()
        self._calc_contour()
        CFence.fence_number += 1

    def width_value(self):
        return 0.3 * self.length

    def _calc_contour(self):
        self.fence_x = []
        self.fence_y = []

        self.fence_x.append(self.length / 2)
        self.fence_y.append(self.width / 2)

        self.fence_x.append(self.length / 2)
        self.fence_y.append(-self.width / 2)

        self.fence_x.append(-self.length / 2)
        self.fence_y.append(-self.width / 2)

        self.fence_x.append(-self.length / 2)
        self.fence_y.append(self.width / 2)

        self.fence_x.append(self.length / 2)
        self.fence_y.append(self.width / 2)

        self.__fence_x, self.__fence_y = CFence._interpolate(self.fence_x, self.fence_y)

    @staticmethod
    def _interpolate(x, y):
        rx, ry = [], []
        dtheta = 0.05
        for i in range(len(x) - 1):
            rx.extend([(1.0 - θ) * x[i] + θ * x[i + 1]  # interpolate the edges
                       for θ in np.arange(0.0, 1.0, dtheta)])
            ry.extend([(1.0 - θ) * y[i] + θ * y[i + 1]
                       for θ in np.arange(0.0, 1.0, dtheta)])
        return rx, ry

    def calc_global_contour(self):
        gx = [(ix * np.cos(self.heading) + iy * np.sin(self.heading)) +
              self.x_base for (ix, iy) in zip(self.__fence_x, self.__fence_y)]
        gy = [(ix * np.sin(self.heading) - iy * np.cos(self.heading)) +
              self.y_base for (ix, iy) in zip(self.__fence_x, self.__fence_y)]
        return gx, gy

    def detection(self, human, plt):
        self.fence_contact(human)

    def fence_contact(self, human):
        if self.contact_flag(human):
            fence_vector = np.array([self.x_base, self.y_base])
            human_vector = CFence.unit_vector(np.array([1, human.heading]))
            rot = (CFence.angle_between(fence_vector, human_vector))
            x_ = - human_vector[0] * np.cos(2 * rot) - (-human_vector[1]) * np.sin(2 * rot)
            y_ = - human_vector[0] * np.sin(2 * rot) + (-human_vector[1]) * np.cos(2 * rot)
            new_human = CFence.unit_vector(np.array([x_, y_]))
            rot_new = (CFence.angle_between(fence_vector, new_human))
            if np.abs(np.pi - rot_new - rot) > 0.1:
                x_ = - human_vector[0] * np.cos(-2 * rot) - (-human_vector[1]) * np.sin(-2 * rot)
                y_ = - human_vector[0] * np.sin(-2 * rot) + (-human_vector[1]) * np.cos(-2 * rot)
            human_vector = CFence.unit_vector(np.array([x_, y_]))
            human.heading = math.atan2(human_vector[1], human_vector[0])

    def contact_flag(self, human):
        flag = False
        in_range = np.hypot(human.x - self.x_base, human.y - self.y_base) < np.hypot(self.length, self.width)
        if in_range:
            corner_x = [(ix * np.cos(self.heading) + iy * np.sin(self.heading)) +
                        self.x_base for (ix, iy) in zip(self.fence_x[0:4], self.fence_y[0:4])]
            corner_y = [(ix * np.sin(self.heading) - iy * np.cos(self.heading)) +
                        self.y_base for (ix, iy) in zip(self.fence_x[0:4], self.fence_y[0:4])]
            gx, gy = human.standing_area()
            for vx, vy in zip(gx, gy):
                if CFence.check(corner_x, corner_y, vx, vy):
                    flag = True
                    break
        return flag

    def cover_area(self):
        dimension = self.length * self.width
        if np.hypot(self.x_base, self.y_base) < 1:
            cover_area = 0.5 * self.length * np.hypot(self.x_base, self.y_base) + dimension
            return cover_area
        else:
            return dimension

    def plot(self, fig):
        fig.plot(self.x_base, self.y_base, ".y")
        gx, gy = self.calc_global_contour()
        plt.plot(gx, gy, "--y")
        plt.text(self.x_base, self.y_base, self.name)

    @staticmethod
    def unit_vector(vector):
        return vector / np.linalg.norm(vector)

    @staticmethod
    def angle_between(v1, v2):
        v1_u = CFence.unit_vector(v1)
        v2_u = CFence.unit_vector(v2)
        return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

    @staticmethod
    def area(x1, y1, x2, y2, x3, y3):

        return abs((x1 * (y2 - y3) +
                    x2 * (y3 - y1) +
                    x3 * (y1 - y2)) / 2.0)

    @staticmethod
    def check(corner_x, corner_y, x, y):
        x1 = corner_x[0]
        x2 = corner_x[1]
        x3 = corner_x[2]
        x4 = corner_x[3]
        y1 = corner_y[0]
        y2 = corner_y[1]
        y3 = corner_y[2]
        y4 = corner_y[3]
        A = (CFence.area(x1, y1, x2, y2, x3, y3) + CFence.area(x1, y1, x4, y4, x3, y3))
        A1 = CFence.area(x, y, x1, y1, x2, y2)
        A2 = CFence.area(x, y, x2, y2, x3, y3)
        A3 = CFence.area(x, y, x3, y3, x4, y4)
        A4 = CFence.area(x, y, x1, y1, x4, y4)
        return np.abs(A1 + A2 + A3 + A4 - A) < 0.01

    class Factory:
        @staticmethod
        def create(parameters): return CFence(**parameters)
