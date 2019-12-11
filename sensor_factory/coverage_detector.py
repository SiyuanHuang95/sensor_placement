import numpy as np


class CPoints:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.marked_status = 0


class CCoverageDetector:
    coverage_dict = {}

    # Here define one dictionary to store the dangerous zone grid coverage situation
    # Key is (x,y) point which the sample point on the dangerous zone circle
    # Value represents how many times that point is covered
    def __init__(self, robot_position_x=0, robot_position_y=0, dangerous_zone_radius=1.5, step_length=0.1):
        self.robot_x = robot_position_x
        self.robot_y = robot_position_y
        self.dangerous_zone_radius = dangerous_zone_radius
        self.step_length = step_length
        self.__cover_matrix()

    # init the cover matrix
    def __cover_matrix(self):
        for x in np.arange(-self.dangerous_zone_radius, self.dangerous_zone_radius + self.step_length,
                           self.step_length):
            x = round(x, 3)
            y_pos = round(self.robot_y + np.sqrt(np.square(self.dangerous_zone_radius) - np.square(self.robot_x - x)),
                          3)
            y_neg = round(self.robot_y - np.sqrt(np.square(self.dangerous_zone_radius) - np.square(self.robot_x - x)),
                          3)
            self.coverage_dict[(x, y_pos)] = 0
            self.coverage_dict[(x, y_neg)] = 0

    # when adding one new sensor, update the dictionary
    def cover_update(self, sensor):
        bad_placement_flag = sensor.coverage_dangerous_zone(self.coverage_dict,
                                                            dangerous_zone_radius=self.dangerous_zone_radius)
        return bad_placement_flag
        # print(self.coverage_dict)

    def reset(self):
        self.__cover_matrix()
