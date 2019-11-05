#!/usr/bin/env python
from functools import reduce

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy
import math
import random
from .utils import strip_nan
from .constants import Constants

const = Constants(False)
class MotionLaser:
    def __init__(self):
        self.turn = False
        self.right_avg = 0
        self.average_count = 0
        self.sum_readings = []
        self.history = [const.FORWARD, const.FORWARD, const.FORWARD]
        self.desired_bearing = 0
        self.correction = False
        self.move_and_turn = False

    def callback(self, msg):
        # global average_count
        # # global RATE
        # global sum_readings
        # global turn
        # global move_and_turn
        # global desired_bearing
        # global correction
        # global right_avg
        self.correction = False
        if const.average_count % const.RATE == 0:
            # plt.clf()
            mapped_readings = list(map(lambda x: x / const.RATE, const.sum_readings))
            # plt.plot(mapped_readings)

            # fig.canvas.draw()
            sum_readings = []
            for value in msg.ranges:
                strip_nan(sum_readings, value)
            left = mapped_readings[const.LEFT_LOWER:const.LEFT_UPPER]
            centre_left = mapped_readings[const.CENTRE_LEFT_LOWER: const.CENTRE_LEFT_UPPER]
            centre = mapped_readings[const.CENTRE_LOWER: const.CENTRE_UPPER]
            centre_right = mapped_readings[const.CENTRE_RIGHT_LOWER:const.CENTRE_RIGHT_UPPER]
            right = mapped_readings[const.RIGHT_LOWER:const.RIGHT_UPPER]

            # CALCUATE AVERAGE READINGS
            left_avg = reduce(lambda a, b: a + b, left) / len(left)
            centre_left_avg = reduce(lambda a, b: a + b, centre_left) / len(centre_left)
            centre_avg = reduce(lambda a, b: a + b, centre) / len(centre)
            centre_right_avg = reduce(lambda a, b: a + b, centre_right) / len(centre_right)
            right_avg = reduce(lambda a, b: a + b, right) / len(right)
            print(left_avg, centre_avg, right_avg)
            print("right_avg: "+str(right_avg))
            avg_data = []
            # pad averages for graphing
            for i in range(len(mapped_readings)):
                if (i < const.LEFT_LOWER):
                    avg_data.append(0)
                elif (i < const.LEFT_UPPER):
                    avg_data.append(left_avg)
                elif (i < const.CENTRE_LEFT_LOWER):
                    avg_data.append(0)
                elif (i < const.CENTRE_RIGHT_UPPER):
                    avg_data.append(centre_avg)
                elif (i < const.RIGHT_LOWER):
                    avg_data.append(0)
                elif (i < const.RIGHT_UPPER):
                    avg_data.append(right_avg)

            # SPACE FORWARD?
            if centre_avg <= const.FRONT_MIN or centre_right_avg <= 0.3:  # turn
                # SPACE RIGHT?
                print("no space front or right, pivoting left")
                turn = True
                desired_bearing = const.LEFT
                move_and_turn = False
            else:
                # SPACE RIGHT?
                if right_avg >= const.RIGHT_MIN:
                    print("space right, turning")
                    self.turn = True
                    self.move_and_turn = True
                    self.desired_bearing = const.RIGHT
                elif right_avg < const.RIGHT_OPTIMAL:
                    print("Too close, turning left")
                    self.turn = True
                    self.move_and_turn = True
                    self.correction = True
                    self.desired_bearing = const.LEFT
                elif const.RIGHT_OPTIMAL < right_avg < const.RIGHT_MIN:
                    print("Too far, turning right")
                    self.turn = True
                    self.move_and_turn = True
                    self.correction = True
                    self.desired_bearing = const.RIGHT
                else:
                    self.turn = False
                    self.desired_bearing = const.FORWARD
                    self.move_and_turn = False

            # plt.plot(avg_data)
            # fig.canvas.draw()
        else:
            temp_values = []
            for value in msg.ranges:
                strip_nan(temp_values, value)

            self.sum_readings = [x + y for x, y in zip(temp_values, const.sum_readings)]
        const.average_count += 1

    def talker(self):
        sub = rospy.Subscriber('/base_scan', LaserScan, self.callback)
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
            if turn and current_bearing < abs(TURN_SCALAR * self.desired_bearing):
                # base_data.angular.z = desired_bearing / 360000
                turn_adjustment = 1
                if self.correction:
                    turn_adjustment = 0.2
                else:
                    turn_adjustment = 1
                if self.desired_bearing > 0:
                    base_data.angular.z = 0.25 * turn_adjustment
                else:
                    base_data.angular.z = -0.25 * turn_adjustment * ((self.right_avg*self.right_avg)/2.25)
                    print(str(self.right_avg)+','+str(turn_adjustment))
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
        m = MotionLaser()
        m.talker()
    except rospy.ROSInterruptException:
        pass
