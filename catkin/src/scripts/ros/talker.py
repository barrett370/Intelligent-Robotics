#!/usr/bin/env python


import rospy  # pyre-ignore
from std_msgs.msg import String  # pyre-ignore
import datetime


def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)  # pyre-ignore
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        hello_str = 'Hello, World at %s' % datetime.datetime.now()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
