from geometry_msgs.msg import Pose, PoseArray, Quaternion
from pf_base import PFLocaliserBase
import math
import rospy

from util import rotateQuaternion, getHeading
from random import random, gauss

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
        self.ODOM_ROTATION_NOISE = 0 # Odometry model rotation noise
        self.ODOM_TRANSLATION_NOISE = 0 # Odometry model x axis (forward) noise
        self.ODOM_DRIFT_NOISE = 0 # Odometry model y axis (side-to-side) noise
 
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
        self.particlecloud = PoseArray() # what do i populate this with 
        newPose = Pose()
        noise_placeholder = 10
        INIT_HEADING = 0 	# Initial orientation of robot (radians)
        for i in range(500):
            #need to generate noise in noise placeholder in the loop with gaussian
            noise_placeholder = gauss(0,1)
            newPose.pose.pose.position.x = initialpose.pose.pose.position.x + noise_placeholder
            newPose.pose.pose.position.y = initialpose.pose.pose.position.y + noise_placeholder
            newPose.pose.pose.position.z = initialpose.pose.pose.position.z + noise_placeholder
            newPose.pose.pose.orientation = rotateQuaternion(Quaternion(w=1.0), INIT_HEADING)  
            # add to particle cloud
        
        return self.particlecloud # not sure about this
        #pass

 
    
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
        pass
