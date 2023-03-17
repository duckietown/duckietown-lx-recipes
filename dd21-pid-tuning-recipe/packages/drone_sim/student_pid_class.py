from __future__ import division


class PID:
    """
    This is your PID class! Be sure to read the docstrings carefully and fill in all methods of this class!
    """

    def __init__(self, kp, ki, kd, k):
        """
        Here is where you will initialize all of the instance variables you might need!

        **IMPORTANT** Be sure to follow the following naming conventions for your control term variables:

        P term: _p
        I term: _i
        D term: _d

        This will ensure that your class works correctly with the rest of the drone's code stack.

        :param kp: The proportional gain constant
        :param ki: The integral gain constant
        :param kd: The derivative gain constant
        :param k: The offset constant that will be added to the sum of the P, I, and D control terms
        """

        pass

    def step(self, err, dt):
        """
        This method will get called at each sensor update, and should return
        a throttle command output to control the altitude of the drone. This is where
        your actual PID calculations will occur. You should implement the discrete version
        of the PID control function.

        :param err: The current error (difference between the setpoint and drone's current altitude) in meters.
                    For example, if the drone was 10 cm below the setpoint, err would be 0.1
        :param dt: The time (in seconds) elapsed between measurements of the process variable (the drone's altitude)
        :returns: You should restrict your output to be between 1100 and 1900. This is a PWM command, which will be
            sent to the SkyLine's throttle channel
        """

        return 1100 + 1000*err

    def reset(self):
        """
        This method will get called when the simulation is reset (by pressing 'r') or when the real drone transitions
        from armed mode to flying mode. You will want to reset the PID terms so that previously stored values will
        not affect the current calculations (think about what this entails)!
        """

        pass
