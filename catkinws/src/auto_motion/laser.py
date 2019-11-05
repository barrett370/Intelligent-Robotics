from functools import reduce

from .sensor_response import Response
from .utils import strip_nan

class Laser:
    def __init__(self, constants):
        self.average_count = 0
        # self.left_avg = 0
        # self.centre_left_avg = 0
        # self.centre_avg = 0
        # self.centre_right_avg = 0
        # self.right_avg = 0
        # self.left = []
        # self.centre_left = []
        # self.centre = []
        # self.centre_right = []
        # self.right = []
        self.sum_readings = []
        self.const = constants

    def input(self, msg):
        temp_values = []
        for value in msg.ranges:
            strip_nan(temp_values, value)

        self.sum_readings = [x + y for x, y in zip(temp_values, self.sum_readings)]

        self.average_count += 1

    def output(self):
        mapped_readings = list(map(lambda x: x / self.const.RATE, self.sum_readings))
        # plt.plot(mapped_readings)

        # fig.canvas.draw()
        left = mapped_readings[self.const.LEFT_LOWER:self.const.LEFT_UPPER]
        centre_left = mapped_readings[self.const.CENTRE_LEFT_LOWER: self.const.CENTRE_LEFT_UPPER]
        centre = mapped_readings[self.const.CENTRE_LOWER: self.const.CENTRE_UPPER]
        centre_right = mapped_readings[self.const.CENTRE_RIGHT_LOWER:self.const.CENTRE_RIGHT_UPPER]
        right = mapped_readings[self.const.RIGHT_LOWER:self.const.RIGHT_UPPER]

        # CALCUATE AVERAGE READINGS
        left_avg = reduce(lambda a, b: a + b, left) / len(left)
        centre_left_avg = reduce(lambda a, b: a + b, centre_left) / len(centre_left)
        centre_avg = reduce(lambda a, b: a + b, centre) / len(centre)
        centre_right_avg = reduce(lambda a, b: a + b, centre_right) / len(centre_right)
        right_avg = reduce(lambda a, b: a + b, right) / len(right)
        print(left_avg, centre_avg, right_avg)
        print("right_avg: " + str(right_avg))
        avg_data = []
        # pad averages for graphing
        for i in range(len(mapped_readings)):
            if (i < self.const.LEFT_LOWER):
                avg_data.append(0)
            elif (i < self.const.LEFT_UPPER):
                avg_data.append(left_avg)
            elif (i < self.const.CENTRE_LEFT_LOWER):
                avg_data.append(0)
            elif (i < self.const.CENTRE_RIGHT_UPPER):
                avg_data.append(centre_avg)
            elif (i < self.const.RIGHT_LOWER):
                avg_data.append(0)
            elif (i < self.const.RIGHT_UPPER):
                avg_data.append(right_avg)

        # SPACE FORWARD?
        out = Response(centre_avg, centre_left_avg, centre_right_avg, left_avg, right_avg)
        self.sum_readings = []
        # if centre_avg <= self.const.FRONT_MIN or centre_right_avg <= 0.3:  # turn
        #     # SPACE RIGHT?
        #     print("no space front or right, pivoting left")
        #     out['turn'] = True
        #     out['desired_bearing'] = self.const.LEFT
        #     out['move_and_turn'] = False
        #     # self.turn = True
        #     # self.desired_bearing = self.const.LEFT
        #     # self.move_and_turn = False
        # else:
        #     # SPACE RIGHT?
        #     if right_avg >= self.const.RIGHT_MIN:
        #         print("space right, turning")
        #         out['turn'] = True
        #         out['desired_bearing'] = self.const.RIGHT
        #         out['move_and_turn'] = True
        #         # self.turn = True
        #         # self.move_and_turn = True
        #         # self.desired_bearing = self.const.RIGHT
        #     elif right_avg < self.const.RIGHT_OPTIMAL:
        #         print("Too close, turning left")
        #         out['turn'] = True
        #         out['desired_bearing'] = self.const.LEFT
        #         out['move_and_turn'] = True
        #         out['correction'] = True
        #         # self.turn = True
        #         # self.move_and_turn = True
        #         # self.correction = True
        #         # self.desired_bearing = self.const.LEFT
        #     elif self.const.RIGHT_OPTIMAL < right_avg < self.const.RIGHT_MIN:
        #         print("Too far, turning right")
        #         out['turn'] = True
        #         out['desired_bearing'] = self.const.LEFT
        #         out['move_and_turn'] = True
        #         out['correction'] = True
        #         # self.turn = True
        #         # self.move_and_turn = True
        #         # self.correction = True
        #         # self.desired_bearing = self.const.RIGHT
        #     else:
        #         out['turn'] = False
        #         out['desired_bearing'] = self.const.FORWARD
        #         out['move_and_turn'] = False
        #         # self.turn = False
        #         # self.desired_bearing = self.const.FORWARD
        #         # self.move_and_turn = False
        return out
