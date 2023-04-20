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


# -------------- read poses and initialize particle box --------------------
poses = read_pose_file('particle_filter_data.txt', 100)

box = ParticleBox(poses)

# ---------------- set up figure and animation ------------------
fig = plt.figure()
fig.subplots_adjust(left=0, right=1, bottom=0, top=1)
ax = fig.add_subplot(111, aspect='equal', autoscale_on=False, xlim=(0, 10), ylim=(0, 10))

# particles holds the locations of the particles
particles, = ax.plot([], [], 'bo', ms=6)

# ------------------ start animation ----------------------
ani = animation.FuncAnimation(fig, animate, interval=200, blit=False, init_func=init)

# ani.save('name.mp4', fps=10, extra_args=['-vcodec', 'libx264'])

plt.show()





