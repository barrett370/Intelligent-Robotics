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

def systematic_resampling(S, M):
    S_n = []
    cdf = [S[0][1]]
    for i in range(1, M):
        cdf.append(cdf[i - 1] + S[i][1])
    U = []
    U.append(np.random.uniform(0, (1 / M), 1))
    for i in range(M):
        for j in range(M):
            while U[j] > cdf[i]:
                i += 1
            S_n.append(S[i])
            U.append(U[j] + (1 / M))

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
        self.RANDOM_FRAC = 0.25

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
        # p_cloud = PoseArray()s
        print(initialpose.pose.pose.position.x,
              initialpose.pose.pose.position.y)
        # self.particlecloud = PoseArray()  # populated with 500 poses
        # noiseValue = 10
        # INIT_HEADING = 0 	# Initial orientation of robot (radians)
        noise_value = 0.9
        self.p_cloud = self.rand_particles(1000)
        # for i in range(1000):
        #     new_pose = Pose()
        #     # need to generate noise in noise placeholder in the loop with gaussian
        #     # random_gauss = gauss(0, 7)

        #     # mu and kappa are set to 0 to generate a random value in a distribution between 0 and 2pi radians
        #     generated_angle = random.vonmisesvariate(mu=0, kappa=0)
        #     new_pose.position.x = initialpose.pose.pose.position.x + (gauss(0, 7)*noise_value)
        #     new_pose.position.y = initialpose.pose.pose.position.y + (gauss(0, 7)*noise_value)
        #     new_pose.position.z = initialpose.pose.pose.position.z
        #     # z wont have any noise
        #     # new_pose.orientation = rotateQuaternion(
        #     #     new_pose.orientation, generated_angle)
        #     new_pose.orientation = Quaternion(new_pose.position.x, new_pose.position.y, new_pose.position.z,
        #                                     generated_angle)
        #     # add to particle cloud
        #     self.p_cloud.poses.append(
        #         new_pose)  # append particle cloud to
        # print(new_pose)

        print("Initialised particle cloud")
        # print(self.p_cloud)
        # self.particlecloud = deepcopy(self.p_cloud)
        # self.p_cloud = self.rand_particles(self.NUM_PARTICLES)
        return self.p_cloud  # returns the particle cloud now populated with poses

    def update_particle_cloud(self, scan):
        samples = []
        scan = filter_nan(scan)
        for particle in self.particlecloud.poses:
            samples.append((particle, self.sensor_model.get_weight(scan, particle)))
        sorted_samples = sorted(samples, key=lambda x: x[1], reverse=True)
        top_sorted_samples = sorted_samples[0:int((1-self.RANDOM_FRAC) * self.NUM_PARTICLES)]
        rand_particles = self.rand_particles(self.NUM_PARTICLES * self.RANDOM_FRAC)
        re_top_sorted_samples = self.resample_v2(top_sorted_samples)
        new_particles = PoseArray()
        for sample in re_top_sorted_samples:
            new_particles.poses.append(sample[0])

        new_particles.poses += rand_particles.poses
        assert (len(new_particles.poses)==self.NUM_PARTICLES)
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

        # Work out the average of the coords
        particles = self.particlecloud.poses
        return self.particle_cluster(particles)

    def particle_cluster(self, particles):
        euclidean_dists = []
        f_euc_dist = lambda p: (math.sqrt(math.pow(p.position.x, 2) + math.pow(p.position.y, 2)))
        # can convert to disctionary if this proves too inefficient
        for particle in particles:
            euclidean_dists.append(f_euc_dist(particle))
        mean_euc_dist = np.mean(euclidean_dists)
        sd_euc_dist = np.std(euclidean_dists)
        if sd_euc_dist > 1000:  # tweak value
            keep_particles = []
            for particle in particles:
                euc_dist = f_euc_dist(particle)
                if mean_euc_dist - sd_euc_dist < euc_dist < mean_euc_dist + euc_dist:
                    keep_particles.append(particle)
            self.particle_cluster(keep_particles)
        else:
            # for particle in particles:
            #     if f_euc_dist(particle) == mean_euc_dist:
            #         return particle
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
            # print(av_ang_x, av_ang_y, av_ang_z, av_ang_w)
            # av_angle = rotateQuaternion(
            #     Quaternion(), av_ang_w)
            av_angle = Quaternion(av_ang_x / len(angles), av_ang_y / len(angles), av_ang_z / len(angles),
                                  av_ang_w / len(angles))

            # av_ang = Quaternion(w=av_ang_w)
            # print("Estimated position as")
            est_pose.position.x = np.mean(xs)
            est_pose.position.y = np.mean(ys)
            est_pose.orientation = av_angle
            # est_pose.header.frame_id = "/map"

            # print(est_pose)
            return est_pose
