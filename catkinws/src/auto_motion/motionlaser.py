#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import constants
import laser
import sonar

const = constants.Constants(False)

class MotionLaser:
    def __init__(self):
        self.turn = False
        self.right_avg = 0
        self.average_count = 0
        self.sum_readings = []
        self.sonar_sum_readings = []
        self.history = [const.FORWARD, const.FORWARD, const.FORWARD]
        self.desired_bearing = 0
        self.correction = False
        self.move_and_turn = False
        self.mapped_readings = []
        self.sonar_mapped_readings = []
        self.laser = laser.Laser(const)
        self.laser_output = None
        self.sonar = sonar.Sonar(const)
        self.sonar_output = None

    def laser_callback(self, msg):
        self.correction = False
        if self.average_count % const.RATE == 0:
            self.laser_output = self.laser.output()
        else:
            self.laser.input(msg)

    def sonar_callback(self, msg):
        if self.average_count % const.RATE == 0:
            self.sonar_output = self.laser.output()
        else:
            self.sonar.input(msg)

    def talker(self):
        sub = rospy.Subscriber('/base_scan', LaserScan, self.laser_callback)
        print('subscribed to /scan')
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
        print('setup publisher to cmd_vel')
        rospy.init_node('Mover', anonymous=True)
        print('setup node')
        rate = rospy.Rate(const.HZ)  # 10hz
        base_data = Twist()
        current_bearing = 0
        TURN_SCALAR = 1000.0
        while not rospy.is_shutdown():

            if self.laser_output.centre_avg <= const.FRONT_MIN or self.laser_output.centre_right_avg <= 0.3:  # turn
                # SPACE RIGHT?
                print("LASER: no space front or right, pivoting left")
                self.turn = True
                self.desired_bearing = const.LEFT
                self.move_and_turn = False
            elif self.sonar_output.centre_avg <= 0.5 or self.sonar_output.centre_right_avg <= 0.5:
                print("SONAR: no space front or right, pivoting left")
                print(self.sonar_output)
                self.turn = True
                self.desired_bearing = const.LEFT
                self.move_and_turn = False
            else:
                # SPACE RIGHT?
                if self.laser_output.right_avg >= const.RIGHT_MIN:
                    print("space right, turning")
                    self.turn = True
                    self.move_and_turn = True
                    self.desired_bearing = const.RIGHT
                elif self.laser_output.right_avg < const.RIGHT_OPTIMAL:
                    print("Too close, turning left")
                    self.turn = True
                    self.move_and_turn = True
                    self.correction = True
                    self.desired_bearing = const.LEFT
                elif const.RIGHT_OPTIMAL < self.laser_output.right_avg < const.RIGHT_MIN:
                    print("Too far, turning right")
                    self.turn = True
                    self.move_and_turn = True
                    self.correction = True
                    self.desired_bearing = const.RIGHT
                else:
                    self.turn = False
                    self.desired_bearing = const.FORWARD
                    self.move_and_turn = False
            if self.turn and current_bearing < abs(TURN_SCALAR * self.desired_bearing):
                # base_data.angular.z = desired_bearing / 360000
                turn_adjustment = 1
                if self.correction:
                    turn_adjustment = 0.2
                else:
                    turn_adjustment = 1
                if self.desired_bearing > 0:
                    base_data.angular.z = 0.25 * turn_adjustment
                else:
                    base_data.angular.z = -0.25 * turn_adjustment * (
                            (self.laser_output.right_avg * self.laser_output.right_avg) / 2.25)
                    print(str(self.laser_output.right_avg) + ',' + str(turn_adjustment))
                # base_data.angular.z  = 0.002*desired_bearing
                if self.move_and_turn:
                    base_data.linear.x = 0.25
                else:
                    # base_data.linear.x = 0
                    base_data.linear.x = 0.0
                current_bearing = current_bearing + 1
            else:
                base_data.angular.z = 0
                base_data.linear.x = 0.2
                current_bearing = 0
                turn = False
            pub.publish(base_data)
            rate.sleep()

if __name__ == '__main__':
    try:
        m = MotionLaser()
        m.talker()
    except rospy.ROSInterruptException:
        pass
