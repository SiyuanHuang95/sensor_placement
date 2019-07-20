import math


class CSensor(object):
    sensor_number = 0

    def __init__(self, name='sensor', x=0, y=0, heading=0):
        self.name = name
        self.x_base = x
        self.y_base = y
        if heading == 0:
            self.heading = math.pi - math.atan2(self.x_base, self.y_base)
        else:
            self.heading = heading
        CSensor.sensor_number += 1

    def set_price(self, price=1):
        self.price = price

    def __str__(self, print_all=False):
        if print_all:
            return " ".join(str(items) for items in (self.__dict__.items()))
        else:
            return self.name

    def detection(self, human, plt):
        raise NotImplementedError

    def plot(self, fig):
        raise NotImplementedError

    def cover_area(self):
        raise NotImplementedError
