from .student_pid_class import PID
import numpy as np
import matplotlib.pyplot as plt
import sys


class VerticalDrone:
    """
    This class simulates a drone that can only move in one dimension, the vertical
    direction. You will use this simulation to test and tune your PID class before you
    transfer it onto your real drone. Do not modify any of the code in this file.
    """

    def __init__(self, pid_terms=[0, 0, 0, 0],
                step_size=0, latency=0, drag_coeff=0, mass=460, sensor_noise=0):
        self.setpoint = 0.5

        self.x = 0
        self.xsp = 0
        self.g = -9.81
        self.step_size = step_size / 100.
        self.latency = latency
        self.drag_coeff = drag_coeff
        self.mass = mass
        self.sensor_noise = sensor_noise / 100.
        self.pid = PID(pid_terms[0], pid_terms[1], pid_terms[2], pid_terms[3])
        self.reset()

    def step(self, t):
        if self.lastt is not None:
            dt = t - self.lastt  # update time
        else:
            dt = 1

        self.lastt = t

        self.error = self.setpoint - self.z # update error
        self.interror += self.error
        self.deriverror = self.lasterror - self.error
        self.lasterror = self.error

        noise = np.random.normal(scale=self.sensor_noise) if self.sensor_noise > 0 else 0
        pwm = self.pid.step(self.error + noise, dt)  # calc forces

        self.latent_thrusts.append(self.pwm_to_thrust(pwm))
        thrust = self.latent_thrusts.pop(0)
        drag = - np.sign(self.vz) * self.vz ** 2 * self.drag_coeff
        # TODO - add ground effect 

        self.az = self.g + (drag + thrust) / self.mass  # update drone
        self.vz += + self.az * dt
        self.z += self.vz * dt
        if self.z <= 0:
            self.z = 0
            self.vz = 0

        self.times.append(t)
        self.errors.append(self.error)
        self.z_list.append(self.z)
        
    def update_setpoint(self,height):
        self.setpoint = height

    def update_params(self, drag_coeff, latency, noise):
        """
        :param drag_coeff 
        :param latency
        :param noise
        """
        self.latency = latency
        self.drag_coeff = drag_coeff
        self.sensor_noise = noise/100

    def reset(self):
        self.lastt = 0
        self.times = []
        self.errors = []
        self.z_list = []
        self.latent_thrusts = [1100] * self.latency
        self.z = 0
        self.vz = 0
        self.az = 0
        self.interror = 0
        self.lasterror = 0
        self.pid.reset()

    def simulate(self,start_time=0.0,end_time=5.0):
        timevec = np.linspace(start_time,end_time,1000)
        self.reset()

        for t in timevec:
            self.step(t)

    def plot_errors(self):
        fig = plt.gcf()
        fig.clear()
        plt.plot(self.times,self.errors)
        plt.title("Error history")
        plt.show()

    def plot_step_response(self):
        fig = plt.gcf()
        fig.clear()
        ax = plt.axes()
        
        ax.plot(self.times,self.z_list)
        ax.set_xlim(0,10)
        ax.hlines(self.setpoint,0,1000,linestyles='--',color='r')
        
        ax.set_title("Step response")
        plt.show()

    def pwm_to_thrust(self, pwm):
        max_thrust = 420 * 4 * 9.81  # max thrust in newtons
        pwm_min = 1100.
        pwm_max = 1900.
        pwm = max(pwm_min, min(pwm, pwm_max))  # bound the pwm between 1100 and 1900
        throttle_fraction = (pwm - pwm_min) / (pwm_max - pwm_min)  # rescale between 0 and 1

        return throttle_fraction * max_thrust


    def press(self, event):
        if event.key == 'up':
            self.setpoint += self.step_size
        elif event.key == 'down':
            if self.setpoint > 0: self.setpoint -= self.step_size
        elif event.key == 'r':
            self.reset()
        elif event.key == 'q':
            plt.close('all')
            sys.exit(0)
