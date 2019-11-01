#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import numpy

RATE = 10
average_count = 0
sum_readings= []

fig = plt.gcf()
fig.show()
fig.canvas.draw()
simMode = True
if simMode:
    left_lower = 0
    left_upper = 50
    centre_left_lower = 200
    centre_left_upper = 224
    centre_lower = 224
    centre_upper = 275
    centre_right_lower = 276
    centre_right_upper = 300
    right_lower = 450
    right_upper = 500
else:
    left_lower = 25
    left_upper = 125
    centre_left_lower = 274
    centre_left_upper = 324
    centre_lower = 325
    centre_upper = 375
    centre_right_lower = 376
    centre_right_upper = 426
    right_lower = 675
    right_upper = 700

def callback(msg):
    global average_count
    global RATE
    global sum_readings
    if(average_count%RATE ==0 ):
        plt.clf()
        plt.plot(map(lambda x: x/RATE,sum_readings))
        fig.canvas.draw()
        sum_readings =[]
        for value in msg.ranges:
            if str(value) == "nan":
                val = 5.5
            else:
                val = value
            
        sum_readings.append(val)
    else: 
        temp_values = []
        for value in msg.ranges:
            if str(value) == "nan":
                val = 5.5
            else:
                val = value
            temp_values.append(val)
            
                
        sum_readings = [x + y for x, y in zip(temp_values, sum_readings)]
    average_count += 1



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
