import numpy as np
import math
import copy

NUM_PARTICLES = 100
UPDATE_VARIANCE = 0.1
TARGET = "<choose a location with both x,y within 0-10>"


class Particle:
	### TODO: Implement this class###
    """
    Implement class to hold a particle in your filter. Should store its position (x,y)
    and its weight. Note that the weight field must
    be called "weight." Once optimized in problem 6, it should store the set of
    all weights and poses as numpy arrays. Depending on how you implement the
    optimization, it may also need fields or methods for retrieving the particle's
    own weight and pose.
    """
    def __init__(self, x, y):
		###TODO: your code here###
        pass


class ParticleSet:
    ### TODO: Implement this class###
    """
    TODO: Implement class to hold the set of particles and the methods which will operate on them.
    You may want to also store the target pose here as well. Remember, the goal
    is for the (x,y) positions of the set of particles to converge on the
    target pose. You must implement a motion model, measurement_model, and
    resampling method.
    """

    def __init__(self, num_particles):
        """
        Takes the number of particles as input and creates a set of particles at
        random positions. Note: the list of particles must be called "particles"
        and the particle poses should be within the range 0<x<10 and 10<y<10
        """
		### TODO: your code here###

    def particle_motion(self):
        """
        Adds some random Gaussian noise to the x and y positions of each
        particle in the filter. Be sure that the noise is different for each
        particle. Use UPDATE_VARIANCE for the variance when you sample from the
        Gaussian
        Hint: try numpy.random.normal
        """
		### TODO: your code here###

    def weight_particles(self):
        """
        Sets the weight of each particle inversely proportional to the
        particle’s distance from the desired pose. ie, if a particle is close
        to the desired pose, it should have a high weight, and if a particle
        is far from the desired pose, it should have a low weight. Recall that
        X is inversely proportional to Y if X = 1/Y. This weighting scheme will
        ensure that particles closest to the target have higher weights, and
        thus are resampled with a higher probability.
        """
		### TODO: your code here###

    def resample_particles(self):
        """
        Uses the particle weights to resample the particles and create a new set.
        The goal is for each particle to be resampled with a probability that
        corresponds to its weight, therefore the particle set converges on the
        desired pose (particles closest to the target pose are more likely to be
        resampled). This method is tricky and has been implemented for you.
        """

        new_particles = []

        # get the list of particle weights, and normalize them
        weights = np.array([p.weight for p in self.particles])
        weights_sum = np.sum(weights)
        normal_weights = weights / float(weights_sum)

        # choose samples from old particle set based on weights
        samples = np.random.multinomial(self.num_particles, normal_weights)
        for i, count in enumerate(samples):
            for _ in range(count):
                p = copy.deepcopy(self.particles[i])
                p.weight = PROB_THRESHOLD
                new_particles.append(p)

        # replace the set of particles
        self.particles = new_particles



###############################################################################
# Do not edit below this line
###############################################################################

particles = ParticleSet(NUM_PARTICLES)

pose_path = 'particle_filter_data.txt'

try:
    pp = open(pose_path, 'w')
    for _ in range(500):
        particles.particle_motion()
        particles.weight_particles()
        particles.resample_particles()

        for p in particles.particles:
            pp.write(str(p.pose[0]) + '\n' + str(p.pose[1]) + '\n')
finally:
    pp.close()

