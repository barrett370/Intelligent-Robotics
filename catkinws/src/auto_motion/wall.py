#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

SPEED = -0.2
FPS = 20

class wall_hugger():

	def __init__(self, distance=1.0, threshold=2.0, wall_tolerance=0.2, bearing_tolerance=2):
		self.left = 0.0
		self.bearing = 0
		self.distance = distance
		self.threshold = threshold
		self.wall_tolerance = wall_tolerance
		self.bearing_tolerance = bearing_tolerance

		rospy.init_node('wall_hugger', anonymous=True)
		rate = rospy.Rate(FPS)

		velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.callback)

		vel_msg = Twist()

		while not rospy.is_shutdown():

			vel_msg.linear.x = SPEED
			vel_msg.angular.z = 0

			separation = self.left - self.distance
			keep_wall_at = 90
			velocity_publisher.publish(vel_msg)

			rate.sleep()

	def angle_to_range(self, angle):
		return int((8.0 / 3.0) * (angle + 135))

	def callback(self, msg):
		smallest = 100
		index = 0
		for i in range(0, 135):
			left = msg.ranges[self.angle_to_range(i)]
			if left < smallest:
				smallest = left
				index = i
		self.left = smallest
		self.bearing = index
                print(self.bearing)

if __name__ == '__main__':
	try:
		wall_hugger()
	except rospy.ROSInterruptException: pass
