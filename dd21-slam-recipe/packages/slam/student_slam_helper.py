"""
slam_helper.py

Implements fastSLAM for the pidrone
"""

import numpy as np
import math
import utils
import copy
import cv2

# set this to true to write your SLAM output to a file to be animated
WRITE_TO_FILE = True

# ----- camera parameters DO NOT EDIT ------- #
CAMERA_SCALE = 290.
CAMERA_WIDTH = 320.
CAMERA_HEIGHT = 240.
MATCH_RATIO = 0.75

# ----- SLAM parameters ------- #
PROB_THRESHOLD = 0.002
KEYFRAME_DIST_THRESHOLD = CAMERA_HEIGHT / 4
KEYFRAME_YAW_THRESHOLD = 0.175

# ----- edit to where you want the SLAM data written --------- #
path = 'pose_data.txt'


class Particle:
    """
    attributes:
    pose:           a list giving the robot's position (x, y, z, yaw)
    landmarks:      a list of landmark objects
    weight:         the current weight for the particle
    """

    def __init__(self, x, y, z, yaw):
        self.pose = [x, y, z, yaw]
        self.landmarks = []
        self.weight = PROB_THRESHOLD

    def __str__(self):
        return "Pose: " + str(self.pose) + " Weight: " + str(self.weight)


class FastSLAM:
    def __init__(self):
        self.particles = None
        self.num_particles = None
        self.weight = PROB_THRESHOLD

        self.z = 0
        self.perceptual_range = 0.0

        # key points and descriptors from the most recent keyframe
        self.key_kp = None
        self.key_des = None

        if WRITE_TO_FILE:
            self.file = open(path, 'w')

        # --------------- openCV parameters --------------------- #
        index_params = dict(algorithm=6, table_number=6,
                            key_size=12, multi_probe_level=1)
        search_params = dict(checks=50)
        self.matcher = cv2.FlannBasedMatcher(index_params, search_params)

        # ------- parameters for noise on observations ----------- #
        self.sigma_d = 1
        self.sigma_p = .30
        self.sigma_observation = np.array(
            [[self.sigma_d ** 2, 0], [0, self.sigma_p ** 2]])

        # ------- parameters for noise on motion updates --------- #
        sigma_vx = .001
        sigma_vy = .001
        sigma_vz = 0.0
        sigma_yaw = 0.0001
        self.covariance_motion = np.array([[sigma_vx ** 2, 0, 0, 0],
                                           [0, sigma_vy ** 2, 0, 0],
                                           [0, 0, sigma_vz ** 2, 0],
                                           [0, 0, 0, sigma_yaw ** 2]])

    def generate_particles(self, num_particles):
        """
        Creates the initial set of particles for SLAM
        Each particle should be initialized at (0,0) since we build the map relative to the drone's
        initial position, but with some noise.

        :param num_particles: the number of particles to generate
        """

        ##############################################################################
        # [x] Initialize the set of particles for FASTSLAM with x,y poses around 0,
        #      z initialized to self.z, and yaw intialized to pi. Be sure to add noise
        #      to the z,y, and yaw initial estimates!
        # Be sure to call it 'self.particles'

        self.particles = []

        standard_dev = 0.02

        for i in range(num_particles):
            self.particles.append(Particle(np.random.normal(0, standard_dev), np.random.normal(
                0, standard_dev), self.z, np.random.normal(math.pi, standard_dev)))

        ##############################################################################

        # Reset SLAM variables in case of restart
        self.num_particles = num_particles
        self.key_kp, self.key_des = None, None
        self.weight = PROB_THRESHOLD

        return estimate_pose(self.particles)

    def run(self, z, prev_kp, prev_des, kp, des):
        """
        Applies an iteration of the FastSLAM algorithm

        :param z: the new infrared height estimate for the drone
        :param prev_kp, prev_des: the keypoints and descriptors from the previous frame
        :param kp, des: the current keypoints and descriptors
        """

        # ---------------- FILE WRITING CODE DO NOT EDIT ----------------------------- #
        if WRITE_TO_FILE:
            if self.particles is not None and self.particles[0].landmarks != []:
                # write the poses of all the particles
                self.file.write(str([[p.pose[0], p.pose[1]]
                                for p in self.particles]) + '\n')
                # write the landmark poses for the first particle
                # for l in self.particles[0].landmarks:
                # print(l.x,l.y)
                self.file.write(
                    str([[lm.x, lm.y] for lm in self.particles[0].landmarks]) + '\n')
                # write the GLOBAL poses of all the currently observed features relative to first particle
                poses = []
                for k in kp:
                    kp_x, kp_y = k[0], k[1]
                    # key point y is measured from the top left
                    kp_y = CAMERA_HEIGHT - kp_y

                    dx = kp_x - CAMERA_WIDTH / 2
                    dy = kp_y - CAMERA_HEIGHT / 2
                    pose = [self.pixel_to_meter(dx) + self.particles[0].pose[0],
                            self.pixel_to_meter(dy) + self.particles[0].pose[1]]
                    poses.append(pose)
                self.file.write(str(poses) + '\n')
        # ------------------------------------------------------------------------------ #

        self.z = z

        # reflect that the drone can see more or less if its height changes
        self.update_perceptual_range()

        ##################################################################################
        # TODO implement the remainder of this method
        #
        # Just as in localization, you will need to update each particle with the motion
        # estimate given by "utils.compute_transform"
        # Then, you should call self.detect_keyframe to determine whether to perform
        # a map update
        ##################################################################################

        transform = utils.compute_transform(
            self.matcher, prev_kp, prev_des, kp, des)

        if(transform is not None):

            for particle in self.particles:
                x = self.meter_to_pixel(particle.pose[0])-transform[0, 2]
                y = self.meter_to_pixel(particle.pose[1])+transform[1, 2]
                yaw = particle.pose[3] - \
                    np.arctan2(transform[1, 0], transform[0, 0])

        # x = -transform[0, 2]
        # y = transform[1, 2]
        # yaw = -np.arctan2(transform[1, 0], transform[0, 0])
                self.predict_particle(particle, x, y, yaw)

        # [x, y, z, yaw]

        self.detect_keyframe(kp, des)

        return estimate_pose(self.particles), self.weight

    def predict_particle(self, particle, x, y, yaw):
        """
        Updates this particle's position according to the transformation from prev frame to this one.
        The robot's new position is determined based on the control, with added noise to account for control error

        :param particle:  the particle containing the robot's position information
        :param x, y, yaw: the "controls" computed by transforming the previous camera frame to this one to
                          find the dislplacement, NOTE these are measured in pixels!
        """
        ##############################################################################
        # TODO use the x,y,yaw transformation between the previous frame and this one
        #      to update the pose of particle. Be sure to add different noise for each
        #      dimension and you may set the z pose of each particle to self.z

        ##############################################################################

        x = self.pixel_to_meter(x)
        y = self.pixel_to_meter(y)
        yaw = utils.adjust_angle(yaw)

        particle.pose = np.random.multivariate_normal(
            [x, y, self.z, yaw], self.covariance_motion)

    def detect_keyframe(self, kp, des):
        """
        Performs a map update if
        1) there is not a previous keyframe
        2) the distance between the previous keyframe and the current frame is above a threshold
        3) we cannot transform between the previus frame and this one.

        :param kp, des: the lists of keypoints and descriptors
        """
        ##############################################################################
        # TODO implement this method, please call self.update_map to update the map
        #
        # Note that if transform is the transformation matrix from utils.compute_transform
        #       then: x = -transform[0, 2]
        #             y = transform[1,2]
        #             yaw = -np.arctan2(transform[1, 0], transform[0, 0])
        ##############################################################################

        # if self.key_des is not None or self.key_kp is not None:
        #     self.update_map(kp, des)
        #     return

        transform = utils.compute_transform(
            self.matcher, self.key_kp, self.key_des, kp, des)

        if transform is None:
            self.update_map(kp, des)
            return

        x = -transform[0, 2]
        y = transform[1, 2]
        yaw = -np.arctan2(transform[1, 0], transform[0, 0])

        if abs(x) > KEYFRAME_DIST_THRESHOLD or abs(y) > KEYFRAME_DIST_THRESHOLD or abs(yaw) > KEYFRAME_YAW_THRESHOLD:
            print("updating map because of movement")
            self.update_map(kp, des)
            return

    def update_map(self, kp, des):
        """
        Perform a map update on every particle, sets self.weight to the average particle weight,
        resamples the set of particles, and sets the new keyframe to the current frame.

        Please do not edit this method

        :param kp, des: the lists of keypoints and descriptors
        """
        # TODO implement this method according to the docstring
        # note: we have provided self.get_average_weight
        self.weight = 0
        for particle in self.particles:
            self.update_particle(particle, kp, des)
            self.weight += particle.weight

        self.weight /= len(self.particles)

        self.resample_particles()

        self.key_des = des
        self.key_kp = kp

    def update_particle(self, particle, keypoints, descriptors):
        """
        Associate observed keypoints with an old particle's landmark set and update the EKF
        Increment the landmark's counter if it finds a match, otherwise add a new landmark
        Decrement the counter of a landmark which is close to this particle's pose and not observed
        :param particle: the old particle to perform the data association on
        :param keypoints, descriptors: the lists of currently observed keypoints and descriptors
        """

        particle.weight = PROB_THRESHOLD

        # if this particle has no landmarks, make all measurements into landmarks
        if len(particle.landmarks) == 0:
            for kp, des in zip(keypoints, descriptors):
                utils.add_landmark(
                    particle, kp, des, self.sigma_observation, self.kp_to_measurement)
                particle.weight += math.log(PROB_THRESHOLD)
        else:
            # find particle's landmarks in a close range, close_landmarks holds tuples of the landmark and its index
            close_landmarks = []
            for i, lm in enumerate(particle.landmarks):
                if utils.distance(lm.x, lm.y, particle.pose[0], particle.pose[1]) <= self.perceptual_range * 1.2:
                    close_landmarks.append((lm, i))

            part_descriptors, matched_landmarks = None, None
            if len(close_landmarks) != 0:
                # get the descriptors of relevant landmarks
                part_descriptors = [lm[0].des for lm in close_landmarks]

                # we will set to true indices where a landmark is matched
                matched_landmarks = [False] * len(close_landmarks)

            for kp, des in zip(keypoints, descriptors):
                # length 1 list of the most likely match between this descriptor and all the particle's descriptors
                match = None
                if part_descriptors is not None:
                    match = self.matcher.knnMatch(
                        np.array([des]), np.array(part_descriptors), k=2)
                    match = match[0]  # peel off the outer brackets

                # there was no match (short circuiting!)
                # try:
                #     print(match[0].distance)
                #     print(MATCH_RATIO * match[1].distance)
                # except Exception as e:
                #     print(e)
                if match is None or len(match) < 2 or match[0].distance > MATCH_RATIO * match[1].distance:
                    utils.add_landmark(
                        particle, kp, des, self.sigma_observation, self.kp_to_measurement)
                    # 'punish' this particle since new landmarks decrease certainty
                    particle.weight += math.log(PROB_THRESHOLD)
                else:
                    # get the index of the matched landmark in close_landmarks
                    close_index = match[0].trainIdx
                    # print("match=true")
                    matched_landmarks[close_index] = True
                    dated_landmark = close_landmarks[close_index]

                    # print(dated_landmark)

                    # update the original landmark in this particle
                    updated_landmark = utils.update_landmark(particle, dated_landmark[0], kp, des,
                                                             self.sigma_observation, self.kp_to_measurement)
                    particle.landmarks[dated_landmark[1]] = updated_landmark

                    # 'reward' this particles since revisiting landmarks increases certainty
                    particle.weight += math.log(scale_weight(
                        match[0].distance, match[1].distance))

            if matched_landmarks is not None:
                # increment counter for revisited particles, and decrement counter for non-revisited particles
                removed = []
                for i, match in enumerate(matched_landmarks):
                    tup = close_landmarks[i]
                    lm = particle.landmarks[tup[1]]
                    if match:
                        lm.counter += 1
                    else:
                        lm.counter -= 1
                        # 'punish' this particle for having a dubious landmark
                        particle.weight += math.log(0.1*PROB_THRESHOLD)
                        if lm.counter < 0:
                            removed.append(lm)

                for rm in removed:
                    particle.landmarks.remove(rm)

    def get_average_weight(self):
        """
        the average weight of all the particles

        :param particles: the set of particles whose weight will be averaged
        """
        return np.sum([p.weight for p in self.particles]) / float(self.num_particles)

    def pixel_to_meter(self, px):
        """
        uses the camera scale to convert pixel measurements into meters

        :param px: the distance in pixels to convert
        """
        ##############################################################################
        # [x] paste your code from localization to convert pixels to meters
        ##############################################################################
        return px*self.z/float(CAMERA_SCALE)

    def meter_to_pixel(self, m):
        return m/(self.z/float(CAMERA_SCALE))

    def kp_to_measurement(self, kp):
        """
        Computes the range and bearing from the center of the camera frame to (kp.x, kp.y)
        bearing is in (-pi/2, pi/2) measured in the standard math way

        :param kp: they keypoint to measure from
        """
        kp_x, kp_y = kp[0], kp[1]
        # key point y is measured from the top left
        kp_y = CAMERA_HEIGHT - kp_y

        dx = kp_x - CAMERA_WIDTH / 2
        dy = kp_y - CAMERA_HEIGHT / 2
        dist = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))
        dist = self.pixel_to_meter(dist)
        bearing = math.atan2(dy, dx)

        return dist, bearing

    def update_perceptual_range(self):
        """
        computes the perceptual range of the drone: the distance from the center of the frame to the
        width-wise border of the frame
        """
        self.perceptual_range = self.pixel_to_meter(CAMERA_WIDTH / 2)

    def resample_particles(self):
        """
        resample particles according to their weight

        :param particles: the set of particles to resample
        :return: a new set of particles, resampled with replacement according to their weight
        """

        weight_sum = 0.0
        new_particles = []
        normal_weights = np.array([])

        weights = [p.weight for p in self.particles]
        lowest_weight = min(weights)

        for w in weights:
            x = 1 - (w / lowest_weight)
            if x == 0:
                x = PROB_THRESHOLD
            normal_weights = np.append(normal_weights, x)
            weight_sum += x

        normal_weights /= weight_sum
        samples = np.random.multinomial(self.num_particles, normal_weights)
        for i, count in enumerate(samples):
            for _ in range(count):
                new_particles.append(copy.deepcopy(self.particles[i]))

        self.particles = new_particles


def scale_weight(match0, match1):
    """
    uses the distances of the two best matches to provide a weight scaled between 0 and 1

    :param match0: the hamming distance of the first best match
    :param match1: the hamming distance of the second best match
    """
    scaled = (match1 - match0) / float(match1)
    if scaled == 0:
        scaled = PROB_THRESHOLD
    return scaled


def estimate_pose(particles):
    """
    retrieves the drone's estimated position by summing each particle's pose estimate multiplied
    by its weight

    Some mathematical motivation: imagine that the pose estimate of the drone along each degree of
    freedom is represented by a random variable. We find the pose estimate by taking the expectation
    of the random variable:
    Expectation[X] = sum over all x in X of p(x) * x
    where p(x) is the weight of a particle and x is the pose of a particle

    :param particles: the set of particles to estimate a position for
    :return: the estimated pose [x,y,z,yaw]
    """

    weight_sum = 0
    for particle in particles:
        weight_sum += particle.weight

    x = 0
    y = 0
    z = 0
    yaw = 0
    #   self.pose = [x, y, z, yaw]
    for particle in particles:
        x += particle.pose[0]*particle.weight/weight_sum
        y += particle.pose[1]*particle.weight/weight_sum
        z += particle.pose[2]*particle.weight/weight_sum
        yaw += particle.pose[3]*particle.weight/weight_sum

    return [x, y, z, yaw]
