from geometry_msgs.msg import Pose, PoseArray, Quaternion
from pf_base import PFLocaliserBase
import math
import rospy

from util import rotateQuaternion, getHeading
from random import gauss
import random
import numpy as np
from time import time


# You need to make changes ONLY in pf.py file for completing the localisation package.
# If you want to change any of the inherited parameters (parameters inherited from the parent class PFLocaliserBase), it is
# best to do so from the child class itself (i.e. in PFLocaliser class in pf.py).
# However, you may play with different values for parameters in the other files (eg. sensor_model.py) for conducting experiments.

def systematic_resampling(S, M):
    S_n = []
    cdf = [S[0][1]]
    for i in range(1, M):
        cdf[i] = cdf[i - 1] + S[i][1]
    U = []
    U[0] = random.uniform(0, (1 / M), 1)
    for i in range(M):
        for j in range(M):
            while U[j] > cdf[i]:
                i += 1
            S_n.append(S[i])
            U[j + 1] = U[j] + (1 / M)

    return S_n


class PFLocaliser(PFLocaliserBase):

    def __init__(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()

        # ----- Set motion model parameters
        # These need to be changed to non-zero values
        self.particle_cloud = PoseArray()
        self.ODOM_ROTATION_NOISE = 0  # Odometry model rotation noise
        # Odometry model x axis (forward) noise
        self.ODOM_TRANSLATION_NOISE = 0
        self.ODOM_DRIFT_NOISE = 0  # Odometry model y axis (side-to-side) noise

        # ----- Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 20  # Number of readings to predict

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
        self.particle_cloud = PoseArray()  # populated with 500 poses
        # noiseValue = 10
        # INIT_HEADING = 0 	# Initial orientation of robot (radians)
        for i in range(500):
            new_pose = Pose()
            # need to generate noise in noise placeholder in the loop with gaussian
            random_gauss = gauss(0, 1)
            noise_value = 1
            # mu and kappa are set to 0 to generate a random value in a distribution between 0 and 2pi radians
            generated_angle = random.vonmisesvariate(mu=0, kappa=0)
            new_pose.position.x = initialpose.pose.pose.position.x + \
                                 random_gauss * self.ODOM_TRANSLATION_NOISE
            new_pose.position.y = initialpose.pose.pose.position.y + \
                                 random_gauss * self.ODOM_DRIFT_NOISE

            new_pose.position.z = initialpose.pose.pose.position.z  # z wont have any noise
            new_pose.orientation = rotateQuaternion(
                Quaternion(w=1.0), generated_angle)
            # add to particle cloud
            self.particle_cloud.poses.append(
                new_pose)  # append particle cloud to
        print("Initialised particle cloud")
        return self.particle_cloud  # returns the particle cloud now populated with poses

    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.

        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update

         """
        S = []
        for particle in self.particle_cloud.poses:  # added .poses as self.particlecloud doesn't seem to be iterable
            S.append((particle, self.sensor_model.get_weight(scan, particle)))
        pass
        S_n = systematic_resampling(S, len(S))
        new_particles = PoseArray()
        for each in S_n:
            new_particles.poses.append(each[0])
        if self.particle_cloud != new_particles:
            print("Particle Cloud updated")
        self.particle_cloud = new_particles

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

        # Work out the average of the coords
        particles = self.particle_cloud.poses
        euclidean_dists = np.array()
        def f_euc_dist(p): return (math.sqrt(math.pow(p.position.x, 2) + math.pow(p.position.y,
                                                                                  2)))  # can convert to disctionary if this proves too inefficient
        for particle in particles:
            np.append(euclidean_dists, f_euc_dist(particle))
        mean_euc_dist = np.mean(euclidean_dists)
        sd_euc_dist = np.std(euclidean_dists)
        if sd_euc_dist > 1:  # tweak value
            keep_particles = PoseArray()
            for particle in particles:
                euc_dist = f_euc_dist(particle)
                if mean_euc_dist - sd_euc_dist < euc_dist < mean_euc_dist + euc_dist:
                    keep_particles.poses.append(particle)
            self.particle_cluster(keep_particles)
        else:
            # for particle in particles:
            #     if f_euc_dist(particle) == mean_euc_dist:
            #         return particle
            xs = np.array()
            ys = np.array()
            angles = np.array()
            for particle in particles:
                np.append(xs, particle.positon.x)
                np.append(ys, particle.position.y)
                np.append(angles, particle.position)
            av_ang_x = 0
            av_ang_y = 0
            av_ang_z = 0
            av_ang_w = 0
            for angle in angles:
                av_ang_x += angle.x
                av_ang_y += angle.y
                av_ang_z += angle.z
                av_ang_w += angle.w

            av_ang = Quaternion(x=av_ang_x, y=av_ang_y, z=av_ang_z, w=av_ang_w)
            print("Estimated position as")
            est_pose =  Pose(np.mean(xs), np.mean(ys), av_ang)
            print(est_pose)
            return est_pose
