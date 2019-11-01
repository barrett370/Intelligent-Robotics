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

RIGHT = -90
LEFT = 90
BACKWARDS = 180
FORWARD = 0
average_count = 0
sum_readings = []
history = [FORWARD, FORWARD, FORWARD]
desired_bearing = 0
RATE = 5
HZ = 2
if simMode:
    left_lower = 0
    left_upper = 50
    centre_left_lower = 200
    centre_left_upper = 224
    centre_lower = 224
    centre_upper = 275
    centre_right_lower = 276
    centre_right_upper = 300
    right_lower = 450
    right_upper = 500
else:
    left_lower = 25
    left_upper = 125
    centre_left_lower = 274
    centre_left_upper = 324
    centre_lower = 325
    centre_upper = 375
    centre_right_lower = 376
    centre_right_upper = 426
    right_lower = 675
    right_upper = 700


def clean_laser_readings(msg):
    prev = 0
    temp_values = []
    for i in range(0, len(msg)):
        for value in msg:
            if value < 1:
                value = prev

            elif str(value) == "nan":
                value = 5.5

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
    global turn
    global flip
    global sum_readings
    global desired_bearing
    global average_count
    laser_val = msg.ranges[::-1]
    # left = laser_val[left_lower: left_upper]
    # left_avg = average_list(left)
    # centre_left = laser_val[centre_left_lower:centre_left_upper]
    # centre_left_avg = average_list(centre_left)
    # centre = laser_val[centre_lower: centre_upper]
    # centre_avg = average_list(centre)
    # centre_right = laser_val[centre_right_lower:centre_right_upper]
    # centre_right_avg = average_list(centre_right)
    # centre_avg = (0.5 * centre_avg) + (0.25 * centre_right_avg) + (0.25 * centre_left_avg)
    # right = laser_val[right_lower: right_upper]
    # right_avg = average_list(right)
    # print(left_avg, centre_avg, right_avg)
    average_count +=1
    print(len(sum_readings))
    if average_count % RATE == 0:
        # print(average_count)
        mapped_readings = map(lambda x: x / RATE, sum_readings)

        # fig.canvas.draw()
        sum_readings = []
        for value in laser_val:
            strip_nan(sum_readings, value)

        left = mapped_readings[left_lower:left_upper]
        centre_left = mapped_readings[centre_left_lower: centre_left_upper]
        centre = mapped_readings[centre_lower: centre_upper]
        centre_right = mapped_readings[centre_right_lower:centre_right_upper]
        right = mapped_readings[right_lower:right_upper]
        # print(left)
        left_avg = reduce(lambda a, b: a + b, left) / len(left)
        centre_left_avg = reduce(lambda a, b: a + b, centre_left) / len(centre_left)
        centre_avg = reduce(lambda a, b: a + b, centre) / len(centre)
        centre_right_avg = reduce(lambda a, b: a + b, centre_right) / len(centre_right)
        right_avg = reduce(lambda a, b: a + b, right) / len(right)
        print(left_avg,centre_avg,right_avg)
        
        # print(random.uniform(0, 1))
        # print msg.ranges[250]
        # check centre and right
        if centre_avg < 1.1:  # turn
            if right_avg > 1.7:
                desired_bearing = RIGHT
                print("turning right 1")
                turn = True
                # turn right
            elif left_avg > 1.7:
                desired_bearing = LEFT
                print("turning left")
                turn = True
                # turn left
            else:
                desired_bearing = BACKWARDS
                print("reversing, beep beep beep")
                turn = True
                # reverse and recurse
        elif centre_avg > 1.1 and right_avg > 4.5 and left_avg > 4.5:
            desired_bearing = FORWARD
            turn = False
            print("Cant find anything, heading forward")
        elif centre_avg > 1.1 and right_avg > 1.7:  # space to the right
            desired_bearing = RIGHT
            print("turning right 2 ")
            turn = True
            # turn right
        elif right_avg < 0.6 < left_avg:
            # turn left
            desired_bearing = LEFT
            turn = True
        elif left_avg < 0.6 < right_avg:
            desired_bearing = RIGHT
            turn = True
        else:
            desired_bearing = FORWARD
            print("keeping on")
            turn = False


    else:
        temp_values = []
        for value in msg.ranges:
            strip_nan(temp_values, value)

        sum_readings = [x + y for x, y in zip(temp_values, sum_readings)]
        # if history[0] == history[2] and desired_bearing == history[1]:  ## Forces a hand if in stalemate
        #     if random.uniform(0, 1) > 0.5:
        #         desired_bearing = history[0]
        #     else:
        #         desired_bearing = FORWARD
        # random.uniform(0, 1)
        # history.append(desired_bearing)
        # history.pop(0)
        # forward
    # if(msg.ranges[250]<1):
    #     print("turn")
    #     if(not turn):
    #         turn = True
    #         if(flip == 1):
    #             flip = -1
    #         else:
    #             flip = 1
    # else:
    # turn = False


def talker():
    sub = rospy.Subscriber('/base_scan', LaserScan, callback)
    print('subscribed to /scan')
    global desired_bearing
    desired_bearing = FORWARD
    global flip
    global turn
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
    print('setup publisher to cmd_vel')
    rospy.init_node('Mover', anonymous=True)
    print('setup node')
    rate = rospy.Rate(HZ)  # 10hz
    base_data = Twist()
    current_bearing = 0
    TURN_SCALAR = 1500.0
    while not rospy.is_shutdown():
        if turn and current_bearing < abs(TURN_SCALAR * desired_bearing):
            base_data.linear.x = 0
            base_data.angular.z = desired_bearing / 180
            current_bearing = current_bearing + 1
        else:
            base_data.angular.z = 0
            base_data.linear.x = 0.25
            current_bearing = 0
            turn = False
        # pub.publish(base_data)

    rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
