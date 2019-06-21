import math
import matplotlib.pyplot as plt


class CRobot(object):
    def __init__(self, name, start_vel=1, start_pos=0):
        self.name = name
        self.x_base = 0
        self.y_base = 0
        self.vel = start_vel
        self.pos = start_pos - math.pi / 2
        self.range = 2.5
        self.__baseRadius = 0.5
        self.eef_x = self.range * math.sin(self.pos)
        self.eef_y = self.range * math.cos(self.pos)

    def update(self):
        self.pos -= self.vel
        self.eef_x = self.range * math.sin(self.pos)
        self.eef_y = self.range * math.cos(self.pos)

    def plot(self, fig):
        # fig.plot(self.x_base,self.y_base,"sr")
        fig.plot(self.eef_x, self.eef_y, "sr")
        base = plt.Circle((self.x_base, self.y_base), self.__baseRadius, color='r', fill=True)
        fig.add_artist(base)
        fig.plot([self.x_base, self.eef_x], [self.y_base, self.eef_y], "-r")
        plt.text(self.x_base - 1, self.y_base + 0.1, self.name)