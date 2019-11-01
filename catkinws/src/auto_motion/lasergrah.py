#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt

fig = plt.gcf()
fig.show()
fig.canvas.draw()

left_lower = 25
left_upper = 125
centre_left_lower = 274
centre_left_upper = 324
centre_lower = 325
centre_upper = 375
centre_right_lower = 376
centre_right_upper = 426
right_lower = 675
right_upper = 699


def average_list(xs):
    sum = 0
    for x in xs:
        if not str(x) == "nan":
            if x < 1:
                sum += x * 0.001
            elif x > 3:
                sum += 3
            else:
                sum += x
    return sum / len(xs)


def callback(msg):
    laser_val =msg.ranges[::-1]
    left = laser_val[left_lower: left_upper]
    left_avg = average_list(left)
    centre_left = laser_val[centre_left_lower:centre_left_upper]
    centre_left_avg = average_list(centre_left)
    centre = laser_val[centre_lower: centre_upper]
    centre_avg = average_list(centre)
    centre_right = laser_val[centre_right_lower:centre_right_upper]
    centre_right_avg = average_list(centre_right)
    centre_avg = (0.5 * centre_avg) + (0.25 * centre_right_avg) + (0.25 * centre_left_avg)
    right = laser_val[right_lower: right_upper]
    right_avg = average_list(right)
    avg_data = []
    for i in range(0,len(laser_val)):
        if(i<left_lower):
            avg_data.append("nan")
        elif(i<left_upper):
            avg_data.append(left_avg)
        elif (i<centre_left_lower):
            avg_data.append("nan")
        elif (i<centre_right_upper):
            avg_data.append(centre_avg)
        elif (i<left_lower):
            avg_data.append("nan")
        elif (i<left_upper):
            avg_data.append(right_avg)
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
    plt.plot(avg_data)
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
