#!/usr/bin/env python
from functools import reduce

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy
import math
import random

simMode = True  # sets the vairables if in sim mode
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
    # Laser groupings
    LEFT_LOWER = 0
    LEFT_UPPER = 50
    CENTRE_LEFT_LOWER = 200
    CENTRE_LEFT_UPPER = 224
    CENTRE_LOWER = 224
    CENTRE_UPPER = 275
    CENTRE_RIGHT_LOWER = 276
    CENTRE_RIGHT_UPPER = 300
    RIGHT_LOWER = 450
    RIGHT_UPPER = 500
else:
    # Laser groupings
    RIGHT_LOWER = 25
    RIGHT_UPPER = 125
    CENTRE_RIGHT_LOWER = 274
    CENTRE_RIGHT_UPPER = 324
    CENTRE_LOWER = 325
    CENTRE_UPPER = 375
    CENTRE_LEFT_LOWER = 376
    CENTRE_LEFT_UPPER = 426
    LEFT_LOWER = 675
    LEFT_UPPER = 700

    # Optimal Ranges
    RIGHT_OPTIMAL = 0.5
    RIGHT_MAX = 2.0
    FRONT_MIN = 3.0


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


# def callback(msg):
#     global turn
#     global flip
#     global sum_readings
#     global desired_bearing
#     global average_count
#     laser_val = msg.ranges[::-1]
#     average_count +=1
#     print(len(sum_readings))
#     if average_count % RATE == 0:
#         # print(average_count)
#         mapped_readings = map(lambda x: x / RATE, sum_readings)

#         # fig.canvas.draw()
#         sum_readings = []
#         for value in laser_val:
#             strip_nan(sum_readings, value)

#         left = mapped_readings[left_lower:left_upper]
#         centre_left = mapped_readings[centre_left_lower: centre_left_upper]
#         centre = mapped_readings[centre_lower: centre_upper]
#         centre_right = mapped_readings[centre_right_lower:centre_right_upper]
#         right = mapped_readings[right_lower:right_upper]
#         # print(left)
#         left_avg = reduce(lambda a, b: a + b, left) / len(left)
#         centre_left_avg = reduce(lambda a, b: a + b, centre_left) / len(centre_left)
#         centre_avg = reduce(lambda a, b: a + b, centre) / len(centre)
#         centre_right_avg = reduce(lambda a, b: a + b, centre_right) / len(centre_right)
#         right_avg = reduce(lambda a, b: a + b, right) / len(right)
#         print(left_avg,centre_avg,right_avg)
        
#         # print(random.uniform(0, 1))
#         # print msg.ranges[250]
#         # check centre and right
#         if centre_avg < 1.1:  # turn
#             if right_avg > 1.7:
#                 desired_bearing = RIGHT
#                 print("turning right 1")
#                 turn = True
#                 # turn right
#             elif left_avg > 1.7:
#                 desired_bearing = LEFT
#                 print("turning left")
#                 turn = True
#                 # turn left
#             else:
#                 desired_bearing = BACKWARDS
#                 print("reversing, beep beep beep")
#                 turn = True
#                 # reverse and recurse
#         elif centre_avg > 1.1 and right_avg > 4.5 and left_avg > 4.5:
#             desired_bearing = FORWARD
#             turn = False
#             print("Cant find anything, heading forward")
#         elif centre_avg > 1.1 and right_avg > 1.7:  # space to the right
#             desired_bearing = RIGHT
#             print("turning right 2 ")
#             turn = True
#             # turn right
#         elif right_avg < 0.6 < left_avg:
#             # turn left
#             desired_bearing = LEFT
#             turn = True
#         elif left_avg < 0.6 < right_avg:
#             desired_bearing = RIGHT
#             turn = True
#         else:
#             desired_bearing = FORWARD
#             print("keeping on")
#             turn = False


#     else:
#         temp_values = []
#         for value in msg.ranges:
#             strip_nan(temp_values, value)

#         sum_readings = [x + y for x, y in zip(temp_values, sum_readings)]


def callback(msg):
    global average_count
    global RATE
    global sum_readings
    if average_count % RATE == 0:
        # plt.clf()
        mapped_readings = map(lambda x: x / RATE, sum_readings)
        # plt.plot(mapped_readings)
        
        #fig.canvas.draw()
        sum_readings = []
        for value in msg.ranges:
            strip_nan(sum_readings, value)
        left = mapped_readings[LEFT_LOWER:LEFT_UPPER]
        centre_left = mapped_readings[CENTRE_LEFT_LOWER: CENTRE_LEFT_UPPER]
        centre = mapped_readings[CENTRE_LOWER: CENTRE_UPPER]
        centre_right = mapped_readings[CENTRE_RIGHT_LOWER:CENTRE_RIGHT_UPPER]
        right = mapped_readings[RIGHT_LOWER:RIGHT_UPPER]

        left_avg = reduce(lambda a, b: a + b, left) / len(left)
        centre_left_avg = reduce(lambda a, b: a + b, centre_left) / len(centre_left)
        centre_avg = reduce(lambda a, b: a + b, centre) / len(centre)
        centre_right_avg = reduce(lambda a, b: a + b, centre_right) / len(centre_right)
        right_avg = reduce(lambda a, b: a + b, right) / len(right)
        print(left_avg,centre_avg,right_avg)
        avg_data = []
        for i in range(len(mapped_readings)):
            # if i < left_lower:
            #     average_data.append(0)
            # elif i == left_lower:
            #     for j in range(len(left)):
            #         average_data.append(left_avg)
            #     i = left_upper
            # elif i < centre_lower:
            #     average_data.append(0)
            # elif i == centre_lower:
            #     for j in range(len(centre)):
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

        if centre_avg < FRONT_MIN:  # turn
            if right_avg > RIGHT_MAX:
                desired_bearing = RIGHT
                print("turning right, \n too little space front, enough space right")
                turn = True
                # turn right
            elif left_avg > 1.7:
                desired_bearing = LEFT
                print("turning left, no space front or right")
                turn = True
                # turn left
            else:
                desired_bearing = BACKWARDS
                print("reversing, beep beep beep")
                turn = True
                # reverse and recurse
        elif centre_avg > FRONT_MIN and right_avg == RIGHT_OPTIMAL and left_avg > 4.5:
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
    global turn
    pub = rospy.Publisher('/cmd_vel_2', Twist, queue_size=100)
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
