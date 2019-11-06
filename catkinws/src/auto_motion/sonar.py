from functools import reduce

import sensor_response
import utils

class Sonar:
    def __init__(self, const):
        self.sum_readings = []
        self.average_count = 0
        self.const = const

    def input(self, msg):
        temp_values = []
        for value in msg.ranges:
            utils.strip_nan(temp_values, value)
        if self.sum_readings == []:
            self.sum_readings = temp_values
        else:
            self.sum_readings = [x + y for x, y in zip(temp_values, self.sum_readings)]

        self.average_count += 1

    def output(self):
        mapped_readings = list(map(lambda x: x / self.const.RATE, self.sum_readings))
        left = mapped_readings[0:1]
        centre_left = mapped_readings[2:3]
        centre = mapped_readings[3:5]
        centre_right = mapped_readings[6]
        right = mapped_readings[7]

        left_avg = reduce(lambda a, b: a + b, left) / len(left)
        centre_left_avg = reduce(lambda a, b: a + b, centre_left) / len(centre_left)
        centre_avg = reduce(lambda a, b: a + b, centre) / len(centre)
        centre_right_avg = reduce(lambda a, b: a + b, centre_right) / len(centre_right)
        right_avg = reduce(lambda a, b: a + b, right) / len(right)

        out = sensor_response.Response(centre_avg, centre_left_avg, centre_right_avg, left_avg, right_avg)
        return out
