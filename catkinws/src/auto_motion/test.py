#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


rospy.init_node("auto_mover", anonymous=False)

rate = rospy.Rate(10)
pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
base_data = Twist()
base_data.angular.x = 0.8
base_data.angular.y = 0.8
base_data.angular.z = 0.8

base_data.linear.x = 0.1
base_data.linear.y = 0.1
base_data.linear.z = 0.1
while not rospy.is_shutdown():
    pub.publish(base_data)
    rate.sleep()
