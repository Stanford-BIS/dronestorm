"""Define a PD controller"""
import numpy as np

class PDController(object):
    """Proportional Derivative controller

    For state x and reference state xr,
    control signal u given by
        u = kp(xr - x) + kd(dxr/dt - dx/dt)

    Parameters
    ----------
    kp: float
        Constant for error
    kd: float
        Constant derivative of error
    ref0: float (default 0)
        zero state - reference signal taken to be relative to this value
    center_error: function or None (default None)
        Function to modify the raw error value
        For example, on a circle the desired angle can be reached by rotating
        clockwise or counter-clockwise. If the error is less than 180 degrees,
        counter-clockwise is the shortest path, but if the error
        is greater than pi radians, clockwise is the shortest path
        Use the center_error function to set the error correction direction
    out_limit: float or None (default None)
        Output control signal is limited to within [-out_limit : out_limit]
        If None, won't be used.
    """
    def __init__(self, kp, kd, ref0=0., center_error=None, out_limit=None):
        self.kp = kp
        self.kd = kd

        self.ref0 = ref0
        self.center_error = center_error
        self.out_limit = out_limit

    def step(self, state, dstate, ref, dref):
        """Update the control signal

        Inputs
        ------
        state : float
            current state
        dstate : float
            time derivative of current state
        ref : float
            reference signal; control input; desired state
        dref : float
            time derivative of reference signal
        """
        # make ref relative to ref0
        ref += self.ref0

        error = ref - state
        if self.center_error is not None:
            error = self.center_error(error)
        derror = dref - dstate

        # compute output
        output = self.kp*error + self.kd*derror
        if self.out_limit is not None:
            output = np.clip(output, -self.out_limit, self.out_limit)

        return output
