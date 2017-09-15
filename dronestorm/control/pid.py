import numpy as np

class PID(object):
    """Proportional Integral Derivative controller

    For state x and reference state xr,
    control signal u given by
        u = kp(xr - x) + kd(dxr/dt - dx/dt) + ki(int(xr - x))
    where int is integral

    Parameters
    ----------
    kp: float
        Constant for error
    kd: float
        Constant derivative of error
    ki: float
        Constant for integral of error
    get_state: function
        Used to get the state value
    get_dstate: function
        Used to get the state time derivative value
    get_ref: function
        Used to get the reference signal value
    get_dref: function or None (default None)
        Used to get the reference signal time derivative value
        If None, assume dref=0
    center_error: function or None (default None)
        Function to modify ythe raw error value
        On a circle, for example, the desired angle can be reached by rotating
        clockwise or counter-clockwise. Unless the error angle is pi radians,
        one direction will be faster. The center_error function can be used
        to set the error correction direction
    i_limit: float or None (default None)
        Integral error is limited to within [-i_limit : i_limit].
        Prevents "integral windup"
        If None, won't be used
    dt: float (default 0.01)
        Timestep of controller. Used to compute the integral
    out_limit: float or None (default None)
        Output control signal is limited to within [-out_limit : out_limit]
        If None, won't be used.
    """
    def __init__(
            self, 
            kp, kd, ki,
            get_state, get_dstate,
            get_ref, get_dref=None,
            center_error=None,
            i_limit=None, dt=0.01,
            out_limit=None):
        self.kp = kp
        self.kd = kd
        self.ki = ki

        self.get_state = get_state
        self.get_dstate = get_dstate
        self.get_ref = get_ref
        self.get_dref = get_dref

        self.center_error = center_error

        self.dt = dt
        self.i_limit = i_limit
        self.out_limit = out_limit

        self.state = 0.
        self.dstate = 0.
        self.ref = 0.
        self.dref = 0.

        self.ierror = 0

    def step(self):
        """Update the control signal

        Reads the current state and reference state and recomputes 
        the control signal
        """
        # get state and reference state
        self.state = self.get_state()
        self.dstate = self.get_dstate()
        self.ref = self.get_ref()
        if self.get_dref is not None:
            self.dref = self.get_dref()

        # compute control components
        error = self.ref - self.state
        if self.center_error is not None:
            error = self.center_error(error)
        derror = self.dref - self.dstate
        self.ierror += error * self.dt
        if self.i_limit is not None:
            self.i_error = np.clip(output, -self.out_limit, self.out_limit)

        # compute output
        output = self.kp * error + self.kd * derror + self.ki * self.ierror
        if self.out_limit is not None:
            output = np.clip(output, -self.out_limit, self.out_limit)

        return output

