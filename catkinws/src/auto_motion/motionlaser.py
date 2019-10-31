#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

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
        sum += x
    return sum / len(xs)


def callback(msg):
    global turn
    global flip
    global desired_bearing
    left = msg.ranges[0, 50]
    left_avg = average_list(left)
    centre = msg.ranges[200, 300]
    centre_avg = average_list(centre)
    right = msg.ranges[450, 500]
    right_avg = average_list(right)
    # print msg.ranges[250]
    # check centre and right
    if centre_avg < 1:  # turn
        if right_avg > 3:
            desired_bearing = RIGHT
            turn = True
            pass  # turn right
        elif left_avg > 3:
            desired_bearing = LEFT
            turn = True
            pass  # turn left
        else:
            desired_bearing = BACKWARDS
            turn = True
            pass  # reverse and recurse
    elif right_avg > 3:  # space to the right
        desired_bearing = RIGHT
        turn = True
        pass  # turn right
    else:
        desired_bearing = FORWARD
        turn = True
        pass  # forward
    turn = False
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


sub = rospy.Subscriber('/scan', LaserScan, callback)
print('subscribed to /scan')


def talker():
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
    TURN_SCALAR = 1.0
    while not rospy.is_shutdown():
        if turn and current_bearing < TURN_SCALAR * desired_bearing:
            base_data.linear.x = 0
            base_data.angular.z = 1
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
