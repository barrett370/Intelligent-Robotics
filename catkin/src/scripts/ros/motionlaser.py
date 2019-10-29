#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

flip = 1
turn = False

def callback(msg):
    global turn
    global flip
    # print msg.ranges[250]
    if(msg.ranges[250]<1):
        print "turn"
        if(not turn):
            turn = True
            if(flip == 1):
                flip = -1
            else:
                flip = 1
    # else:
        # turn = False
    

        

    


sub = rospy.Subscriber('/base_scan', LaserScan, callback)
def talker():
    global flip
    global turn
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
    rospy.init_node('Mover', anonymous=True)
    # rate = rospy.Rate(10) # 10hz
    turnCount = 0
    while not rospy.is_shutdown():
        base_data = Twist()
        if(turn and turnCount < 25000):
            base_data.linear.x = 0
            base_data.angular.z = 1
            turnCount = turnCount + 1
            print turnCount
        else:
            base_data.angular.z = 0
            base_data.linear.x = 1
            turnCount = 0
            turn = False
        pub.publish( base_data )

        
    # rate.sleep()
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass