import math
import matplotlib.pyplot as plt
import numpy as np

area_length = 9


class CHuman(object):
    # represent the human with a circle, which stands for the arm operation, with radius r=1
    def __init__(self, name='worker', start_vel=0.1, start_pos_x=- 4.5, start_pos_y=-3, heading=0):
        self.x = start_pos_x
        self.y = start_pos_y
        self.vel = start_vel
        self.name = name
        self.temp_start_x = np.copy(self.x)
        self.temp_start_y = np.copy(self.y)
        self.arm = 1
        if heading == 0:
            self.heading = self.__heading()
        else:
            self.heading = heading

    def standing_area(self):

        segment = 10
        seg_x = [(self.arm - 0.01) * math.cos(theta) for theta in
                 np.arange(0, np.pi * 2, 2 * np.pi / segment)]
        seg_y = [(self.arm - 0.01) * math.sin(theta) for theta in
                 np.arange(0, np.pi * 2, 2 * np.pi / segment)]

        gx = np.asarray(seg_x) * math.cos(self.heading) + np.asarray(seg_y) * math.sin(self.heading) + self.x
        gy = -np.asarray(seg_x) * math.sin(self.heading) + np.asarray(seg_y) * math.cos(self.heading) + self.y
        return gx, gy

    def __heading_line(self):
        end_point_x = self.x + np.cos(self.heading)
        end_point_y = self.y + np.sin(self.heading)
        return end_point_x, end_point_y

    # Use dynamic model to update the position of the human
    def update(self, dt, omega=0.1, a=0, max_vel=1):
        self.odometry()
        if np.hypot(self.x, self.y) < 1:
            current_pos = np.array([self.x, self.y])
            current_heading = np.array([1, self.heading])
            self.heading += CHuman.angle_between(current_pos, current_heading)
        elif np.hypot(self.x, self.y) < 1.5:
            current_pos = np.array([self.x, self.y])
            current_heading = np.array([1, self.heading])
            self.heading += CHuman.angle_between(current_pos, current_heading) * 0.3
        elif np.hypot(self.x, self.y) < 1.8:
            current_pos = np.array([self.x, self.y])
            current_heading = np.array([1, self.heading])
            self.heading += CHuman.angle_between(current_pos, current_heading) * 0.2

        self.x += self.vel * np.cos(self.heading) * dt
        self.y += self.vel * np.sin(self.heading) * dt
        self.heading += omega * dt
        self.vel += a * dt
        # at first step, always assume to be the constant vel and linear dynamics
        if self.vel > max_vel:
            self.vel = max_vel

    @staticmethod
    def unit_vector(vector):
        return vector / np.linalg.norm(vector)

    @staticmethod
    def angle_between(v1, v2):
        v1_u = CHuman.unit_vector(v1)
        v2_u = CHuman.unit_vector(v2)
        return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

    def __heading(self):
        rough_direction = np.random.choice(['forward', 'backward'], 1, p=[0.7, 0.3])
        if rough_direction == 'forward':
            middle_angle = math.atan2(-self.y, -self.x)
        else:
            middle_angle = math.atan2(self.y, self.x)
        print(rough_direction)
        angle = middle_angle + np.random.uniform(-np.pi/2, np.pi/2)
        return angle

    def odometry(self):
        self.inside_working_area()
        if np.hypot((self.x - self.temp_start_x), (self.y - self.temp_start_y)) > 1:
            # print("change human direction")
            self.change_direction()
            self.temp_start_x = np.copy(self.x)
            self.temp_start_y = np.copy(self.y)

    def change_direction(self):
        self.heading += np.random.normal(0, 0.4)

    def plot(self, fig):
        # fig.plot(self.x,self.y,"sg")
        hx, hy = self.__heading_line()
        plt.text(self.x, self.y, self.name)
        # gx, gy = self.standing_area()
        # fig.plot(gx, gy, "sr")
        fig.plot(hx, hy, "sb")
        human_circle = plt.Circle((self.x, self.y), self.arm, color='lime', fill=True)
        fig.add_artist(human_circle)

    def inside_working_area(self):
        if np.abs(self.x) > area_length or np.abs(self.y) > area_length:
            self.heading = -self.heading * 1.3
            self.x -= self.x/30
            self.y -= self.y/30
