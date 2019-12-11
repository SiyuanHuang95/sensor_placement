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
        # Use the area matrix to mark the placement and avoid the overlapping

    # reset the sensor generation grids
    @staticmethod
    def reset():
        CSpace_control.area = np.ones([CSpace_control.length, CSpace_control.length], dtype=int)

    # check whether the current grid is suitable for the to be generated sensor, e.g. that grid is occupied or not
    @staticmethod
    def check(sensor):
        flag = True
        rows, cols = CSpace_control.get_index(sensor)
        for row in rows:
            for col in cols:
                if col in range(CSpace_control.length) and row in range(CSpace_control.length):
                    if CSpace_control.area[row][col] == 0:
                        flag = False
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
            # lidar will occupy one 3*3 grids
            rows = np.arange(x_-1, x_+1, dtype=int)
            cols = np.arange(y_-1, y_+1, dtype=int)
        elif sensor.name == 'fence':
            # fence will occupy one mean*mean grids
            mean = (sensor.length + sensor.width)//4
            rows = np.arange(x_ - mean, x_ + mean, dtype=int)
            cols = np.arange(y_ - mean, y_ + mean, dtype=int)
        elif sensor.name == 'mat':
            rows = np.arange(x_ - 1, x_ + 1, dtype=int)
            cols = np.arange(y_ - 1, y_ + 1, dtype=int)
        else:
            print("Wrong Sensor Name")
        return rows, cols

