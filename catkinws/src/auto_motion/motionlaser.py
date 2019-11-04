#!/usr/bin/env python
from functools import reduce

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy
import math
import random

simMode = False  # sets the vairables if in sim mode
flip = 1
turn = False

right_avg = 0
RIGHT = -90
LEFT = 90
BACKWARDS = 180
FORWARD = 0
average_count = 0
sum_readings = []
history = [FORWARD, FORWARD, FORWARD]
desired_bearing = 0
RATE = 3  # no. calcs in average
HZ = 10
correction = False
move_and_turn = False
if simMode:
    # Laser groupings
    LEFT_LOWER = 0
    LEFT_UPPER = 50
    CENTRE_LEFT_LOWER = 200
    CENTRE_LEFT_UPPER = 224
    CENTRE_LOWER = 224
    CENTRE_UPPER = 275
    CENTRE_RIGHT_LOWER = 300
    CENTRE_RIGHT_UPPER = 375
    RIGHT_LOWER = 450
    RIGHT_UPPER = 500
    MAX_RANGE = 3
else:
    # Laser groupings
    RIGHT_LOWER = 25
    RIGHT_UPPER = 125
    CENTRE_RIGHT_LOWER = 150
    CENTRE_RIGHT_UPPER = 250
    CENTRE_LOWER = 325
    CENTRE_UPPER = 375
    CENTRE_LEFT_LOWER = 376
    CENTRE_LEFT_UPPER = 426
    LEFT_LOWER = 675
    LEFT_UPPER = 700

    # Optimal Ranges
    RIGHT_OPTIMAL = 0.5
    RIGHT_MAX = 1.5
    RIGHT_MIN = 1.0
    FRONT_MIN = 1.5
    LEFT_MIN = 1.0
    MAX_RANGE = 5.5


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


def callback(msg):
    global average_count
    global RATE
    global sum_readings
    global turn
    global move_and_turn
    global desired_bearing
    global correction
    global right_avg
    correction = False
    if average_count % RATE == 0:
        # plt.clf()
        mapped_readings = map(lambda x: x / RATE, sum_readings)
        # plt.plot(mapped_readings)

        # fig.canvas.draw()
        sum_readings = []
        for value in msg.ranges:
            strip_nan(sum_readings, value)
        left = mapped_readings[LEFT_LOWER:LEFT_UPPER]
        centre_left = mapped_readings[CENTRE_LEFT_LOWER: CENTRE_LEFT_UPPER]
        centre = mapped_readings[CENTRE_LOWER: CENTRE_UPPER]
        centre_right = mapped_readings[CENTRE_RIGHT_LOWER:CENTRE_RIGHT_UPPER]
        right = mapped_readings[RIGHT_LOWER:RIGHT_UPPER]

        # CALCUATE AVERAGE READINGS
        left_avg = reduce(lambda a, b: a + b, left) / len(left)
        centre_left_avg = reduce(lambda a, b: a + b, centre_left) / len(centre_left)
        centre_avg = reduce(lambda a, b: a + b, centre) / len(centre)
        centre_right_avg = reduce(lambda a, b: a + b, centre_right) / len(centre_right)
        right_avg = reduce(lambda a, b: a + b, right) / len(right)
        print(left_avg, centre_avg, right_avg)
        print("right_avg: "+str(right_avg))
        avg_data = []
        # pad advverages for graphing
        for i in range(len(mapped_readings)):
            if (i < LEFT_LOWER):
                avg_data.append(0)
            elif (i < LEFT_UPPER):
                avg_data.append(left_avg)
            elif (i < CENTRE_LEFT_LOWER):
                avg_data.append(0)
            elif (i < CENTRE_RIGHT_UPPER):
                avg_data.append(centre_avg)
            elif (i < RIGHT_LOWER):
                avg_data.append(0)
            elif (i < RIGHT_UPPER):
                avg_data.append(right_avg)

        # SPACE FORWARD?
        if centre_avg <= FRONT_MIN or centre_right_avg <= 0.3:  # turn
            # SPACE RIGHT?
            print("no space front or right, pivoting left")
            turn = True
            desired_bearing = LEFT
            move_and_turn = False
        else:
            # SPACE RIGHT?
            if right_avg >= RIGHT_MIN:
                print("space right, turning")
                turn = True
                move_and_turn = True
                desired_bearing = RIGHT
            elif right_avg < RIGHT_OPTIMAL:
                print("Too close, turning left")
                turn = True
                move_and_turn = True
                correction = True
                desired_bearing = LEFT
            elif RIGHT_OPTIMAL < right_avg < RIGHT_MIN:
                print("Too far, turning right")
                turn = True
                move_and_turn = True
                correction = True
                desired_bearing = RIGHT
            else:
                turn = False
                desired_bearing = FORWARD
                move_and_turn = False

        # plt.plot(avg_data)
        # fig.canvas.draw()
    else:
        temp_values = []
        for value in msg.ranges:
            strip_nan(temp_values, value)

        sum_readings = [x + y for x, y in zip(temp_values, sum_readings)]
    average_count += 1


def talker():
    sub = rospy.Subscriber('/base_scan', LaserScan, callback)
    print('subscribed to /scan')
    global desired_bearing
    desired_bearing = FORWARD
    global flip
    global right_avg
    global turn
    global correction
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
    print('setup publisher to cmd_vel')
    rospy.init_node('Mover', anonymous=True)
    print('setup node')
    rate = rospy.Rate(HZ)  # 10hz
    base_data = Twist()
    current_bearing = 0
    TURN_SCALAR = 1000.0
    while not rospy.is_shutdown():
        if turn and current_bearing < abs(TURN_SCALAR * desired_bearing):
            # base_data.angular.z = desired_bearing / 360000
            turn_adjustment = 1
            if correction:
                turn_adjustment = 0.2
            else:
                turn_adjustment = 1
            if desired_bearing > 0:
                base_data.angular.z = 0.25 * turn_adjustment
            else:
                base_data.angular.z = -0.25 * turn_adjustment * ((right_avg*right_avg)/2.25)
                print(str(right_avg)+','+str(turn_adjustment))
            # base_data.angular.z  = 0.002*desired_bearing
            if move_and_turn:
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
        talker()
    except rospy.ROSInterruptException:
        pass
