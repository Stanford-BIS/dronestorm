"""Attitude controllers"""
from dronestorm.control.pd import PDController
from dronestorm.control.utils import find_min_angle

class AttitudePD(object):
    """PD controller for quadcopter Attitude (roll and pitch)

    Parameters
    ----------
    roll_kp: control constant proportional to roll error
    roll_kd: control constant proportional to roll error derivative
    pitch_kp: control constant proportional to pitch error
    pitch_kd: control constant proportional to pitch error derivative
    """
    def __init__(self, roll_kp, roll_kd, pitch_kp, pitch_kd):
        self.roll_controller = PDController(roll_kp, roll_kd, out_limit=1)
        self.pitch_controller = PDController(pitch_kp, pitch_kd, out_limit=1)

    def step(self, state, dstate, ref, dref):
        """Step the controllers

        Inputs
        ------
        state: [float, float]
            [roll, pitch]
        dstate: [float, float]
            [droll, dpitch]
        ref: [float, float]
            [ref_roll, ref_pitch]
        dref: [float, float]
            [dref_roll, dref_pitch]

        Outputs
        -------
        droll_cmd, dpitch_cmd
        """
        roll, pitch = state
        droll, dpitch = dstate
        ref_roll, ref_pitch = ref
        dref_roll, dref_pitch = dref
        droll_cmd = self.roll_controller.step(roll, droll, ref_roll, dref_roll)
        dpitch_cmd = self.pitch_controller.step(pitch, dpitch, ref_pitch, dref_pitch)
        return droll_cmd, dpitch_cmd
