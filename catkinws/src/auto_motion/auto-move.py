#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class AutoMover():

    def __init__(self):
        self.left = 0.0
        self.bearing = 0
        self.distance = 1.0
        self.wall_threshold = 2.0
        self.wall_tolerance = 0.2
        self.bearing_tolerance = 2.0
        self.speed = 1.0
        self.refresh_rate = 10
        self.turn_angle = 90
        self.keep_wall_at = 90

        pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
        sub = rospy.Subscriber('/scan', LaserScan, self.callback)
        print("Publishing to cmd_vel")
        rospy.init_node("auto_mover", anonymous=False)
        print("Setup custom node, auto_mover")
        rate = rospy.Rate(self.refresh_rate)
        base_data = Twist()

        while not rospy.is_shutdown():
            base_data.linear.x = self.speed
            base_data.angular.z = 0

            distance_from_wall = self.left - self.distance

            if abs(distance_from_wall) < self.wall_threshold:
                print("Wall detected")
                if distance_from_wall > self.wall_tolerance:
                    self.keep_wall_at = 80
                elif distance_from_wall < -self.wall_tolerance:
                    self.keep_wall_at = 100

                correction = abs(self.keep_wall_at - self.bearing) * (5.0 / 90.0)

                if self.bearing < self.keep_wall_at - self.bearing_tolerance:
                    base_data.angular.z = -correction
                elif self.bearing_tolerance > self.keep_wall_at + self.bearing_tolerance:
                    base_data.angular = correction
        pub.publish(base_data)
        rate.sleep()

    def angle_to_range(self, angle):
        return int((8.0 / 3.0) * (angle + 135))

    def callback(self, msg):
        smallest = 100
        index = 0
        for i in range(1, 135):
            left = msg.ranges[self.angle_to_range(i)]
            if left < smallest:
                smallest = left
                index = i
        self.left = smallest
        self.bearing = index


if __name__ == '__main__':
    try:
        AutoMover()
    except rospy.ROSInterruptException:
        pass
