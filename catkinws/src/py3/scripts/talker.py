#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def angle_to_range(angle):
    return int((8.0 / 3.0) * (angle + 135))

def callback(msg):
    smallest = 100
    index = 0
    for i in range(0, 135):
        left = msg.ranges[angle_to_range(i)]
        if left < smallest:
            smallest = left
            index = i
    left = smallest
    bearing = index
    print(bearing)

def talker():
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    scan_subscriber = rospy.Subscriber('/base_scan', LaserScan, callback)
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=False)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        hello_str = 'hello world %s' % str(rospy.get_time)
        # rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass