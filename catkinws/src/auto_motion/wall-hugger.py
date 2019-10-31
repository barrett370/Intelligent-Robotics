#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

SPEED = 1.0
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
        scan_subscriber = rospy.Subscriber('/front/scan', LaserScan, self.callback)

        vel_msg = Twist()

        while not rospy.is_shutdown():

            print("---------------------")
            print("Bearing: " + str(self.bearing))
            print("Distance: " + str(self.left))

            vel_msg.linear.x = SPEED
            vel_msg.angular.z = 0

            separation = self.left - self.distance
            keep_wall_at = 90

            if abs(separation) < self.threshold:
                print("Wall found!")
                if separation > wall_tolerance:
                    keep_wall_at = 80
                    print("Moving closer to wall.")
                elif separation < -wall_tolerance:
                    keep_wall_at = 100
                    print("Moving further from wall.")

                correction = abs(keep_wall_at - self.bearing) * (5.0 / 90.0)

                if self.bearing < keep_wall_at - bearing_tolerance:
                    vel_msg.angular.z = -1 * correction
                    print("Correcting right by " + str(correction))
                elif self.bearing > keep_wall_at + bearing_tolerance:
                    vel_msg.angular.z = correction
                    print("Correcting left by " + str(correction))
                else:
                    print("Cruising...")
            else:
                print("Searching for wall...")

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


if __name__ == '__main__':
    try:
        wall_hugger()
    except rospy.ROSInterruptException:
        pass
