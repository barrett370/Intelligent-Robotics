#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy
flip = 1
turn = False
RIGHT = 90
LEFT = -90
BACKWARDS = 180
FORWARD = 0
desired_bearing = 0


def average_list(xs):
    sum = 0
    for x in xs:
        if x < 1:
            sum += x*0.001
        else:
            sum += x
    return sum / len(xs)


def callback(msg):
    global turn
    global flip
    global desired_bearing
    left = msg.ranges[0: 50]
    left_avg = average_list(left)
    centre = msg.ranges[200: 300]
    centre_avg = average_list(centre)
    right = msg.ranges[450: 500]    
    right_avg = average_list(right)
    print(left_avg , centre_avg ,right_avg)
    # print msg.ranges[250]
    # check centre and right
    if centre_avg < 1:  # turn
        if right_avg > 3:
            desired_bearing = RIGHT
            print("turning right 1")
            turn = True
              # turn right
        elif left_avg > 3:
            desired_bearing = LEFT
            print("turning left")
            turn = True
              # turn left
        else:
            desired_bearing = BACKWARDS
            print("reversing, beep beep beep")
            turn = True
              # reverse and recurse
    elif centre_avg > 1 and  right_avg > 3:  # space to the right
        desired_bearing = RIGHT
        print("turning right 2 ")
        turn = True
          # turn right
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
    TURN_SCALAR = 300.0
    while not rospy.is_shutdown():
        if turn and current_bearing < abs(TURN_SCALAR * desired_bearing):
            base_data.linear.x = 0
            base_data.angular.z = desired_bearing/90
            current_bearing = current_bearing + 1
        else:
            base_data.angular.z = 0
            base_data.linear.x = 0.5
            current_bearing = 0
            turn = False
        pub.publish(base_data)

    # rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
