from copy import deepcopy

from geometry_msgs.msg import Pose, PoseArray, Quaternion, PoseWithCovarianceStamped
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

def systematic_resampling(S, W):
    # cumulative density function
    cdf = []
    # keep track of running total
    cdf_counter = 0
    # create cdf entry for each sample
    for (p, w) in S:
        cdf.append((p, cdf_counter + w / W))
        cdf_counter += w / W
    U = random.uniform(0, 1 / len(S))
    counter = 0
    S_n = PoseArray()
    for j in range(0, len(S)):
        while U > cdf[counter][1]:
            counter += 1
        new_particle = Pose()
        new_particle.position.x = cdf[counter][0].position.x + random.gauss(0, 0.1)
        new_particle.position.y = cdf[counter][0].position.y + random.gauss(0, 0.1)
        new_particle.orientation = rotateQuaternion(Quaternion(w=1),
                                                    getHeading(cdf[counter][0].orientation) + random.gauss(0, 0.05))
        S_n.poses.append(new_particle)
        U += (1 / len(S))

    return S_n


def filter_nan(scan_data):
    out = []
    for i in range(len(scan_data.ranges)):
        if str(scan_data.ranges[i]) == "nan":
            out.append(5.5)
        else:
            out.append(scan_data.ranges[i])
    scan_data.ranges = deepcopy(out)
    return scan_data


class PFLocaliser(PFLocaliserBase):

    def __init__(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()
        self.p_cloud = PoseArray()
        # ----- Set motion model parameters
        # These need to be changed to non-zero values
        # self.particlecloud = PoseArray()
        # self.ODOM_ROTATION_NOISE = 0  # Odometry model rotation noise
        # # Odometry model x axis (forward) noise
        # self.ODOM_TRANSLATION_NOISE = 1
        # self.ODOM_DRIFT_NOISE = 1  # Odometry model y axis (side-to-side) noise
        # # self.estimatedpose = Pose()
        # # ----- Sensor model parameters
        # self.NUMBER_PREDICTED_READINGS = 20  # Number of readings to predict
        self.NUM_PARTICLES = 1000
        self.RANDOM_FRAC = 0.05

    def resample_v2(self, samples):
        new_samples = []
        for sample in samples:
            for i in range(int(sample[1])):
                new_samples.append(sample)
        ret_samples = []
        for i in range(len(samples)):
            ret_samples.append(new_samples[random.randint(0, len(new_samples) - 1)])
        return ret_samples

    def rand_particles(self, N):
        ret = PoseArray()
        width = self.occupancy_map.info.width
        height = self.occupancy_map.info.height
        particles = 0
        while particles < N:
            generated_angle = random.vonmisesvariate(mu=0, kappa=0)
            x = random.randint(0, width - 1)  # maybe use gauss
            y = random.randint(0, height - 1)
            new_pose = Pose()
            new_pose.position.x = x * 0.05
            new_pose.position.y = y * 0.05
            new_pose.orientation = rotateQuaternion(Quaternion(w=1.0), generated_angle)

            if self.occupancy_map.data[x + y * width] == 0:
                ret.poses.append(new_pose)
                particles += 1
        return ret

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
        # self.particlecloud = PoseArray()  # populated with 500 poses
        # noiseValue = 10
        # INIT_HEADING = 0 	# Initial orientation of robot (radians)
        self.p_cloud = self.rand_particles(1000)
        return self.p_cloud  # returns the particle cloud now populated with poses

    def update_particle_cloud(self, scan):
        samples = []
        scan = filter_nan(scan)
        # Create tuple array of Poses and corresponding weights
        for particle in self.particlecloud.poses:
            samples.append((particle, self.sensor_model.get_weight(scan, particle)))
        # sort tuple array by descending weight
        sorted_samples = sorted(samples, key=lambda x: x[1], reverse=True)
        # take top fraction of poses
        top_sorted_samples = sorted_samples[0:int((1 - self.RANDOM_FRAC) * self.NUM_PARTICLES)]
        # calculate the total weight in the particle cloud
        total_weight = sum([x[1] for x in top_sorted_samples])
        # select remaining fraction of particles randomly
        rand_particles = self.rand_particles(self.NUM_PARTICLES * self.RANDOM_FRAC)
        # systematically re-sample from top fraction of previous particles
        re_top_sorted_samples = systematic_resampling(top_sorted_samples, total_weight)
        # re_top_sorted_samples = self.resample_v2(top_sorted_samples)
        new_particles = re_top_sorted_samples

        # concat weighted and resampled poses with random particles
        new_particles.poses += rand_particles.poses

        assert (len(new_particles.poses) == self.NUM_PARTICLES)

        self.particlecloud = new_particles
        # self.particlecloud.header.frame_id = "/map"

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

        # set of recursive method to estimate current pose
        return self.particle_cluster(self.particlecloud.poses)

    def particle_cluster(self, particles):
        euclidean_dists = []
        # function to find euclidean distance of particle from origin
        f_euc_dist = lambda p: (math.sqrt(math.pow(p.position.x, 2) + math.pow(p.position.y, 2)))
        # can convert to dictionary if this proves too inefficient
        # for each particle build array of euclidean distances
        for particle in particles:
            euclidean_dists.append(f_euc_dist(particle))
        # find mean distance from origin
        mean_euc_dist = np.mean(euclidean_dists)
        # find standard devation of euclidean distances
        sd_euc_dist = np.std(euclidean_dists)
        # if standard deviation is above a threshhold, discard outliers and recurse
        if sd_euc_dist > 10:  # tweak value
            keep_particles = []
            for particle in particles:
                euc_dist = f_euc_dist(particle)
                if mean_euc_dist - sd_euc_dist < euc_dist < mean_euc_dist + euc_dist:
                    keep_particles.append(particle)
            return self.particle_cluster(keep_particles)
        # otherwise calculate mean Pose and return that
        else:
            xs = []
            ys = []
            angles = []
            for particle in particles:
                xs.append(particle.position.x)
                ys.append(particle.position.y)
                angles.append(particle.orientation)
            av_ang_x = 0
            av_ang_y = 0
            av_ang_z = 0
            av_ang_w = 0
            for angle in angles:
                av_ang_x += angle.x
                av_ang_y += angle.y
                av_ang_z += angle.z
                av_ang_w += angle.w
            est_pose = Pose()
            av_angle = Quaternion(av_ang_x / len(angles), av_ang_y / len(angles), av_ang_z / len(angles),
                                  av_ang_w / len(angles))

            est_pose.position.x = np.mean(xs)
            est_pose.position.y = np.mean(ys)
            est_pose.orientation = av_angle

            print("Estimated position as")
            print(est_pose)
            return est_pose
