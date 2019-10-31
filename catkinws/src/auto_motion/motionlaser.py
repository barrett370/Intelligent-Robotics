#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy
import math

flip = 1
turn = False
RIGHT = 90
LEFT = -90
BACKWARDS = 180
FORWARD = 0
desired_bearing = 0


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


def callback(msg):
    global turn
    global flip
    global desired_bearing
    left = msg.ranges[0: 50]
    left_avg = average_list(left)
    centre_left = msg.ranges[200:224]
    centre_left_avg = average_list(centre_left)
    centre = msg.ranges[225: 275]
    centre_avg = average_list(centre)
    centre_right = msg.ranges[276:300]
    centre_right_avg = average_list(centre_right)
    centre_avg = (0.5 * centre_avg) + (0.25 * centre_right_avg) + (0.25 * centre_left_avg)
    right = msg.ranges[450: 500]
    right_avg = average_list(right)
    print(left_avg, centre_avg, right_avg)
    # print msg.ranges[250]
    # check centre and right
    if centre_avg < 1.5:  # turn
        if right_avg > 3.5:
            desired_bearing = RIGHT
            print("turning right 1")
            turn = True
            # turn right
        elif left_avg > 3.5:
            desired_bearing = LEFT
            print("turning left")
            turn = True
            # turn left
        else:
            desired_bearing = BACKWARDS
            print("reversing, beep beep beep")
            turn = True
            # reverse and recurse
    elif centre_avg > 1.5 and right_avg > 3.5:  # space to the right
        desired_bearing = RIGHT
        print("turning right 2 ")
        turn = True
        # turn right
    elif right_avg < 0.5 and left_avg > 0.5:
        # turn left 
        desired_bearing = LEFT
        turn = True
    elif left_avg < 0.5 and right_avg > 0.5:
        desired_bearing = RIGHT
        turn = True
    else:
        desired_bearing = FORWARD
        print("keeping on")
        turn = False
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
    rate = rospy.Rate(10)  # 10hz
    base_data = Twist()
    current_bearing = 0
    TURN_SCALAR = 1500.0
    while not rospy.is_shutdown():
        if turn and current_bearing < abs(TURN_SCALAR * desired_bearing):
            base_data.linear.x = 0
            base_data.angular.z = desired_bearing / 90
            current_bearing = current_bearing + 1
        else:
            base_data.angular.z = 0
            base_data.linear.x = 0.25
            current_bearing = 0
            turn = False
        # pub.publish(base_data)

    # rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
