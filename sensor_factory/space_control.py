import numpy as np


class CSpace_control():

    length = 10
    area = np.ones([length, length], dtype=int)

    @staticmethod
    def update(sensor):
        rows, cols = CSpace_control.get_index(sensor)

        for row in rows:
            for col in cols:
                if col in range(CSpace_control.length) and row in range(CSpace_control.length):
                    CSpace_control.area[row][col] = 0

    @staticmethod
    def check(sensor):
        flag = True
        rows, cols = CSpace_control.get_index(sensor)
        for row in rows:
            for col in cols:
                if col in range(CSpace_control.length) and row in range(CSpace_control.length):
                    if CSpace_control.area[row][col] == 0:
                        flag = False
        if flag:
            CSpace_control.update(sensor)
        return flag

    @staticmethod
    def coord_transfer(x, y):
        x_ = x + CSpace_control.length // 2
        y_ = y + CSpace_control.length // 2
        return x_, y_

    @staticmethod
    def get_index(sensor):
        x_, y_ = CSpace_control.coord_transfer(sensor.x_base, sensor.y_base)

        if sensor.name == 'lidar':
            rows = np.arange(x_-1, x_+1, dtype=int)
            cols = np.arange(y_-1, y_+1, dtype=int)
        else:
            mean = (sensor.length + sensor.width)//2
            rows = np.arange(x_ - mean, x_ + mean, dtype=int)
            cols = np.arange(y_ - mean, y_ + mean, dtype=int)
        return rows, cols

