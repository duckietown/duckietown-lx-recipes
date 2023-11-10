import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


def read_pose_file(path, num_particles):
    """
    reads poses of a particle filter from a text file and converts them into
    a nested list

    :param path: the path to a file with particle poses:
    :param num_particles: the number of particles used in the particle filter
    :return: a list of poses formatted according to

    Given m particles and n time steps:

    particles:   0      1             m      |  time:
     poses = [[[x,y], [x,y], ... , [x,y]],   |    0
              [[x,y], [x,y], ... , [x,y]],   |    1
              [[x,y], [x,y], ... , [x,y]],   |    2
               ...                           |
              [[x,y], [x,y], ... , [x,y]]]   |    n
    """
    poses = []

    particle_count = 0
    pair = []
    state = []
    try:
        fp = open(path)
        while True:
            # read a line and check for EOF
            line = fp.readline()
            if not line:
                break
            line = float(line)

            if len(pair) == 1:
                pair.append(line)
                state.append(pair)
                pair = []
                particle_count += 1
            else:
                pair.append(line)

            if particle_count == num_particles:
                poses.append(state)
                state = []
                particle_count = 0
    finally:
        fp.close()

    return poses


class ParticleBox:
    """
    Given m particles and n time steps:

    particles:   0      1             m      |  time:
     poses = [[[x,y], [x,y], ... , [x,y]],   |    0
              [[x,y], [x,y], ... , [x,y]],   |    1
              [[x,y], [x,y], ... , [x,y]],   |    2
               ...                           |
              [[x,y], [x,y], ... , [x,y]]]   |    n

    bounds is the size of the box: [xmin, xmax, ymin, ymax]
    """
    def __init__(self, poses, size = 0.04):
        self.poses = poses
        self.time_step = 0
        self.state = self.poses[self.time_step]
        self.size = size

    def step(self):
        if self.time_step < len(self.poses):
            self.state = self.poses[self.time_step]
            self.time_step += 1

def init():
    """initialize animation"""
    global box
    particles.set_data([], [])
    return particles


def animate(i):
    """perform animation step"""
    global box, ax, fig
    if box.time_step < len(box.poses):
        ms = 2

        # update pieces of the animation
        particles.set_data([x[0] for x in box.state], [x[1] for x in box.state])
        particles.set_markersize(ms)

        box.step()
        return particles
    elif box.time_step == len(box.poses):
        exit()
