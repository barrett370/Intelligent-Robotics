#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt

fig = plt.gcf()
fig.show()
fig.canvas.draw()


def callback(msg):
    plt.clf()
    values=[]
    prev = 0
    for i in range(0,len(msg.ranges),10):
        temp_values=[]
        for value in msg.ranges[i:i+10]:
            if value < 1:
                value = prev
                print('lower')
            elif str(value) == "nan":
                value = 5.5
                print('higher')
            prev = value
            temp_values.append(value)
        values.append(sum(temp_values)/len(temp_values))
    plt.plot(values[::-1])

    fig.canvas.draw()
    


def talker():
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
    rospy.init_node('Mover', anonymous=True)
    sub = rospy.Subscriber('/base_scan', LaserScan, callback)
    print('subscribed to /scan')
    plt.show()
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        base_data = Twist()
        base_data.linear.x = 0.0
        pub.publish( base_data )
        rate.sleep()
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
