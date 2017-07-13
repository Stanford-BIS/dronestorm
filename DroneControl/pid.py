import numpy as np

class PID(object):
    def __init__(
        self, 
        kp, kd, ki,
        get_ref, get_state, get_dstate,
        center_error, out_limit):

        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.out_limit = out_limit
        self.get_ref = get_ref
        self.get_state = get_state
        self.get_dstate = get_dstate
        self.center_error = center_error
        self.ref = 0
        self.state = 0
        self.dstate = 0

    def step(self):
        # get current value
        self.ref = self.get_ref()
        self.state = self.get_state()
        self.dstate = self.get_dstate()

        # compute error
        error = self.center_error(self.ref - self.state)
        derror = self.dstate
        ierror = 0.0 # TODO

        # compute output
        output = np.clip(
            self.kp * error + self.kd * derror + self.ki * ierror,
            -self.out_limit, self.out_limit)

        return output

