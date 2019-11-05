#!/usr/bin/env python
from pprint import pprint
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from p2os_msgs.msg import SonarArray


class Sonar:

    def __init__(self):
        rospy.init_node("sonar", anonymous=False)
        self.sub = rospy.Subscriber('/sonar', SonarArray, self.callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist)
        rate = rospy.Rate(10)
        self.cmd = Twist()
        self.ranges = []
        while not rospy.is_shutdown():
            if len(list(filter(lambda x: x<0.5,self.ranges)))>0:
                print(list(filter(lambda x: x<0.5,self.ranges)))
                self.cmd.linear.x = 0
            else:
                self.cmd.linear.x = 0.1
            self.pub.publish(self.cmd)
            rate.sleep()

    def callback(self, msg):
        pprint(msg.ranges)
        self.ranges = msg.ranges


if __name__ == '__main__':
    Sonar()
