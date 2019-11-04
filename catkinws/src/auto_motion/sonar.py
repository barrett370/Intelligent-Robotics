#!/usr/bin/env python
from pprint import pprint
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
#p2os_msgs/SonarArray
from p2os_msgs.msg import SonarArray

def callback(msg):
    pprint(msg.ranges)
rospy.init_node("sonar", anonymous=False)
sub = rospy.Subscriber('/sonar', SonarArray, callback)
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    rate.sleep()


