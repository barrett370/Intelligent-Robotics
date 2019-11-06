
class Constants:
    def __init__(self, sim_mode):
        self.sim_mode = sim_mode # sets the vairables if in sim mode
        # self.flip = 1
        # self.turn = False

        # self.right_avg = 0
        self.RIGHT = -90
        self.LEFT = 90
        self.BACKWARDS = 180
        self.FORWARD = 0
        # self.average_count = 0
        # self.sum_readings = []
        # self.history = [self.FORWARD, self.FORWARD, self.FORWARD]
        # self.desired_bearing = 0
        self.RATE = 3  # no. calcs in average
        self.HZ = 10
        # correction = False
        # self.move_and_turn = False
        if self.sim_mode:
            # Laser groupings
            self.LEFT_LOWER = 0
            self.LEFT_UPPER = 50
            self.CENTRE_LEFT_LOWER = 200
            self.CENTRE_LEFT_UPPER = 224
            self.CENTRE_LOWER = 224
            self.CENTRE_UPPER = 275
            self.CENTRE_RIGHT_LOWER = 300
            self.CENTRE_RIGHT_UPPER = 375
            self.RIGHT_LOWER = 450
            self.RIGHT_UPPER = 500
            self.MAX_RANGE = 3
        else:
            # Laser groupings
            self.RIGHT_LOWER = 25
            self.RIGHT_UPPER = 125
            self.CENTRE_RIGHT_LOWER = 150
            self.CENTRE_RIGHT_UPPER = 250
            self.CENTRE_LOWER = 325
            self.CENTRE_UPPER = 375
            self.CENTRE_LEFT_LOWER = 376
            self.CENTRE_LEFT_UPPER = 426
            self.LEFT_LOWER = 675
            self.LEFT_UPPER = 700

            # Optimal Ranges
            self.RIGHT_OPTIMAL = 0.5
            self.RIGHT_MAX = 1.5
            self.RIGHT_MIN = 1.0
            self.FRONT_MIN = 1.5
            self.LEFT_MIN = 1.0
            self.MAX_RANGE = 5.5
def clean_laser_readings(msg):
    prev = 0
    temp_values = []
    for i in range(0, len(msg)):
        for value in msg:
            if value < 0.1:
                value = MAX_RANGE
            elif value < 1:
                value = prev

            elif str(value) == "nan":
                value = MAX_RANGE

            prev = value
            temp_values.append(value)
    return temp_values


def is_number(s):
    try:
        float(s)
        return True
    except ValueError:
        return False


def average_list(xs):
    sum = 0
    for x in xs:
        if not str(x) == "nan":
            if x < 1:
                sum += x * 0.001
            elif x > 3:
                sum += 3
            else:
                sum += x
    return sum / len(xs)


def strip_nan(some_readings, value):
    if str(value) == "nan":
        val = 5.5
    else:
        val = value
    some_readings.append(val)
from functools import reduce

from functools import reduce


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
#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from p2os_msgs.msg import SonarArray

const = Constants(False)

class MotionLaser:
    def __init__(self):
        self.turn = False
        self.right_avg = 0
        self.average_count = 0
        self.sum_readings = []
        self.sonar_sum_readings = []
        self.history = [const.FORWARD, const.FORWARD, const.FORWARD]
        self.desired_bearing = 0
        self.correction = False
        self.move_and_turn = False
        self.mapped_readings = []
        self.sonar_mapped_readings = []
        self.laser = Laser(const)
        self.laser_output = None
        self.sonar = Sonar(const)
        self.sonar_output = None

    def laser_callback(self, msg):
        self.correction = False
        if self.average_count % const.RATE == 0:
            self.laser_output = self.laser.output()
        else:
            self.laser.input(msg)

    def sonar_callback(self, msg):
        if self.average_count % const.RATE == 0:
            self.sonar_output = self.laser.output()
        else:
            self.sonar.input(msg)

    def talker(self):
        sub = rospy.Subscriber('/base_scan', LaserScan, self.laser_callback)
        print('subscribed to /scan')
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
        print('setup publisher to cmd_vel')
        rospy.init_node('Mover', anonymous=True)
        print('setup node')
        rate = rospy.Rate(const.HZ)  # 10hz
        base_data = Twist()
        current_bearing = 0
        TURN_SCALAR = 1000.0
        while not rospy.is_shutdown():

            if self.laser_output.centre_avg <= const.FRONT_MIN or self.laser_output.centre_right_avg <= 0.3:  # turn
                # SPACE RIGHT?
                print("LASER: no space front or right, pivoting left")
                self.turn = True
                self.desired_bearing = const.LEFT
                self.move_and_turn = False
            elif self.sonar_output.centre_avg <= 0.5 or self.sonar_output.centre_right_avg <= 0.5:
                print("SONAR: no space front or right, pivoting left")
                print(self.sonar_output)
                self.turn = True
                self.desired_bearing = const.LEFT
                self.move_and_turn = False
            else:
                # SPACE RIGHT?
                if self.laser_output.right_avg >= const.RIGHT_MIN:
                    print("space right, turning")
                    self.turn = True
                    self.move_and_turn = True
                    self.desired_bearing = const.RIGHT
                elif self.laser_output.right_avg < const.RIGHT_OPTIMAL:
                    print("Too close, turning left")
                    self.turn = True
                    self.move_and_turn = True
                    self.correction = True
                    self.desired_bearing = const.LEFT
                elif const.RIGHT_OPTIMAL < self.laser_output.right_avg < const.RIGHT_MIN:
                    print("Too far, turning right")
                    self.turn = True
                    self.move_and_turn = True
                    self.correction = True
                    self.desired_bearing = const.RIGHT
                else:
                    self.turn = False
                    self.desired_bearing = const.FORWARD
                    self.move_and_turn = False
            if self.turn and current_bearing < abs(TURN_SCALAR * self.desired_bearing):
                # base_data.angular.z = desired_bearing / 360000
                turn_adjustment = 1
                if self.correction:
                    turn_adjustment = 0.2
                else:
                    turn_adjustment = 1
                if self.desired_bearing > 0:
                    base_data.angular.z = 0.25 * turn_adjustment
                else:
                    base_data.angular.z = -0.25 * turn_adjustment * (
                            (self.laser_output.right_avg * self.laser_output.right_avg) / 2.25)
                    print(str(self.laser_output.right_avg) + ',' + str(turn_adjustment))
                # base_data.angular.z  = 0.002*desired_bearing
                if self.move_and_turn:
                    base_data.linear.x = 0.25
                else:
                    # base_data.linear.x = 0
                    base_data.linear.x = 0.0
                current_bearing = current_bearing + 1
            else:
                base_data.angular.z = 0
                base_data.linear.x = 0.2
                current_bearing = 0
                turn = False
            pub.publish(base_data)
            rate.sleep()

if __name__ == '__main__':
    try:
        from os import sys, path
        sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))
        m = MotionLaser()
        m.talker()
    except rospy.ROSInterruptException:
        pass
