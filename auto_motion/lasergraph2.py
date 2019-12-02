#!/usr/bin/env python
from functools import reduce

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import numpy

RATE = 10
average_count = 0
sum_readings = []

fig = plt.gcf()
fig.show()
fig.canvas.draw()
simMode = False
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
    right_lower = 575
    right_upper = 675


def callback(msg):
    global average_count
    global RATE
    global sum_readings
    if average_count % RATE == 0:
        plt.clf()
        mapped_readings = map(lambda x: x / RATE, sum_readings)
        plt.plot(mapped_readings)
        
        #fig.canvas.draw()
        sum_readings = []
        for value in msg.ranges:
            strip_nan(sum_readings, value)
        left = mapped_readings[left_lower:left_upper]
        centre_left = mapped_readings[centre_left_lower: centre_left_upper]
        centre = mapped_readings[centre_lower: centre_upper]
        centre_right = mapped_readings[centre_right_lower:centre_right_upper]
        right = mapped_readings[right_lower:right_upper]

        left_avg = reduce(lambda a, b: a + b, left) / len(left)
        centre_left_avg = reduce(lambda a, b: a + b, centre_left) / len(centre_left)
        centre_avg = reduce(lambda a, b: a + b, centre) / len(centre)
        centre_right_avg = reduce(lambda a, b: a + b, centre_right) / len(centre_right)
        right_avg = reduce(lambda a, b: a + b, right) / len(right)
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
            if (i < left_lower):
                avg_data.append(0)
            elif (i < left_upper):
                avg_data.append(left_avg)
            elif (i < centre_left_lower):
                avg_data.append(0)
            elif (i < centre_right_upper):
                avg_data.append(centre_avg)
            elif (i < right_lower):
                avg_data.append(0)
            elif (i < right_upper):
                avg_data.append(right_avg)
        plt.plot(avg_data)
        fig.canvas.draw()
    else:
        temp_values = []
        for value in msg.ranges:
            strip_nan(temp_values, value)

        sum_readings = [x + y for x, y in zip(temp_values, sum_readings)]
    average_count += 1


def strip_nan(some_readings, value):
    if str(value) == "nan":
        val = 5.5
    else:
        val = value
    some_readings.append(val)


def talker():
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
    rospy.init_node('Mover', anonymous=True)
    sub = rospy.Subscriber('/base_scan', LaserScan, callback)
    print('subscribed to /scan')
    plt.show()
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        base_data = Twist()
        base_data.linear.x = 0.0
        pub.publish(base_data)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
