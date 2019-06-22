import math
import matplotlib.pyplot as plt
import numpy as np


class CHuman(object):
    # represent the human with a circle, which stands for the arm operation, with radius r=1
    def __init__(self, name='worker', start_vel=0.1, start_pos_x=- 4.5, start_pos_y=-3, heading=0):
        self.x = start_pos_x
        self.y = start_pos_y
        self.vel = start_vel
        self.name = name
        self.arm = 1
        if heading == 0:
            self.heading = math.pi + math.atan2(self.x, self.y)
        else:
            self.heading = heading

    def standing_area(self):

        segment = 10
        seg_x = [self.arm * math.cos(theta) for theta in np.arange(0, np.pi * 2, 2 * np.pi / segment)]
        seg_y = [self.arm * math.sin(theta) for theta in np.arange(0, np.pi * 2, 2 * np.pi / segment)]

        gx = np.asarray(seg_x) * math.cos(self.heading) + np.asarray(seg_y) * math.sin(self.heading) + self.x
        gy = -np.asarray(seg_x) * math.sin(self.heading) + np.asarray(seg_y) * math.cos(self.heading) + self.y
        return gx, gy

    # Use dynamic model to update the position of the human
    def update(self, dt, omega=0.1, a=0, max_vel=1):
        if np.hypot(self.x, self.y) < 2.5:
            heading_robot = math.pi + math.atan2(self.x, self.y)
            if heading_robot - self.heading > np.pi/12:
                self.heading -= np.pi/100
            else:
                self.heading += np.pi /100
        self.x += self.vel * np.cos(self.heading) * dt
        self.y += self.vel * np.sin(self.heading) * dt
        self.heading += omega * dt
        self.vel += a * dt
        # at first step, always assume to be the constant vel and linear dynamics
        if self.vel > max_vel:
            self.vel = max_vel

    def plot(self, fig):
        # fig.plot(self.x,self.y,"sg")
        plt.text(self.x, self.y, self.name)
        gx, gy = self.standing_area()
        fig.plot(gx, gy, "sr")
        human_circle = plt.Circle((self.x, self.y), self.arm, color='lime', fill=True)
        fig.add_artist(human_circle)
# TODO: Let the human avoid the collision with robot base
