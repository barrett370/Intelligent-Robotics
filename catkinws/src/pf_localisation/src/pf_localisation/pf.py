from geometry_msgs.msg import Pose, PoseArray, Quaternion
from pf_base import PFLocaliserBase
import math
import rospy

from util import rotateQuaternion, getHeading
from random import gauss
import random
from time import time


# You need to make changes ONLY in pf.py file for completing the localisation package.
# If you want to change any of the inherited parameters (parameters inherited from the parent class PFLocaliserBase), it is
# best to do so from the child class itself (i.e. in PFLocaliser class in pf.py).
# However, you may play with different values for parameters in the other files (eg. sensor_model.py) for conducting experiments.

class PFLocaliser(PFLocaliserBase):

    def __init__(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()

        # ----- Set motion model parameters
        # potentially upper bounds? according to jon
        self.ODOM_ROTATION_NOISE = 0  # Odometry model rotation noise
        # Odometry model x axis (forward) noise
        self.ODOM_TRANSLATION_NOISE = 0
        self.ODOM_DRIFT_NOISE = 0  # Odometry model y axis (side-to-side) noise

        # ----- Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 20     # Number of readings to predict

    def initialise_particle_cloud(self, initialpose):
        """
        Set particle cloud to initialpose plus noise

        Called whenever an initialpose message is received (to change the
        starting location of the robot), or a new occupancy_map is received.
        self.particlecloud can be initialised here. Initial pose of the robot
        is also set here.

        :Args:
            | initialpose: the initial pose estimate
        :Return:
            | (geometry_msgs.msg.PoseArray) poses of the particles
        """
        print(initialpose.pose.pose.position.x,
              initialpose.pose.pose.position.y)
        self.particlecloud = PoseArray()  # populated with 500 poses
        newPose = Pose()
        # noiseValue = 10
        # INIT_HEADING = 0 	# Initial orientation of robot (radians)
        for i in range(500):
            # need to generate noise in noise placeholder in the loop with gaussian
            noiseValue = gauss(0, 1)
            # mu and kappa are set to 0 to generate a random value in a distribution between 0 and 2pi radians
            generatedAngle = random.vonmisesvariate(mu=0, kappa=0)
            newPose.position.x = initialpose.pose.pose.position.x + \
                noiseValue * self.ODOM_TRANSLATION_NOISE
            newPose.position.y = initialpose.pose.pose.position.y + \
                noiseValue * self.ODOM_DRIFT_NOISE
            newPose.position.z = initialpose.pose.pose.position.z  # z wont have any noise
            newPose.orientation = rotateQuaternion(
                Quaternion(w=1.0), generatedAngle)
            # add to particle cloud
            self.particlecloud.poses.append(
                newPose)  # append particle cloud to
        return self.particlecloud  # returns the particle cloud now populated with poses

    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.

        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update

         """
        pass

    def estimate_pose(self):
        """
        This should calculate and return an updated robot pose estimate based
        on the particle cloud (self.particlecloud).

        Create new estimated pose, given particle cloud
        E.g. just average the location and orientation values of each of
        the particles and return this.

        Better approximations could be made by doing some simple clustering,
        e.g. taking the average location of half the particles after 
        throwing away any which are outliers

        :Return:
            | (geometry_msgs.msg.Pose) robot's estimated pose.
         """

        sum_x = 0
        sum_y = 0
        self.particlecloud.poses

        # Work out the average of the coords

        for i in range(0, len(self.particlecloud.poses)):
            sum_x += self.particlecloud.poses[i].position.x
            sum_y += self.particlecloud.poses[i].position.y

        average_x = sum_x / 500
        average_y = sum_y / 500

        estimatePose = Pose()
        estimatePose.position.x = average_x
        estimatePose.position.y = average_y
        estimatePose.position.z = self.particlecloud.poses[0].position.z
        # This needs updating
        estimatePose.orientation = self.particlecloud.poses[0].orientation
        return estimatePose
        
        # pass
