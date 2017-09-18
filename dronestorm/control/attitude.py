"""Attitude controllers"""
from .pid import PID

def _min_angle(error):
    """Finds the minimum angular difference between two angles (degrees)"""
    if error > 180:
        error -= 360
    elif error < -180:
        error += 360
    return error

class AttitudePID(object):
    """PID controller for quadcopter Attitude (roll, pitch)

    Parameters
    ----------
    drone_comm: DroneComm instance
        communication object with drone
    roll_gains: {'kp': kp, 'ki': ki, 'kd': kd}
    pitch_gains: {'kp': kp, 'ki': ki, 'kd': kd}
    """
    def __init__(
            self, drone_comm,
            roll_gains, pitch_gains, yaw_gains
            out_roll_limit=1.0, out_pitch_limit=1.0, out_yaw_limit=1.0,
            ):
        self.roll_controller = PID(
            roll_gains['kp'], roll_gains['ki'], roll_gains['kd'],
            lambda:roll0, drone.get_roll, drone.get_droll,
            _min_angle, out_roll_limit)

        self.yaw_controller = PID(
            yaw_gains['kp'], yaw_gains['ki'], yaw_gains['kd'],
            lambda:yaw0, drone.get_yaw, drone.get_dyaw,
            _min_angle, out_yaw_limit)

        self.pitch_controller = PID(
            pitch_gains['kp'], pitch_gains['ki'], pitch_gains['kd'],
            lambda:pitch0, drone.get_pitch, drone.get_dpitch,
            lambda x: x, out_pitch_limit)

    def step(self):
        output_yaw = self.yaw_controller.step()
        output_roll = self.roll_controller.step()
        output_pitch = self.pitch_controller.step()

        self.drone_comm.set_yaw_rate(output_yaw)
        self.drone_comm.set_roll_rate(output_roll)
        self.drone_comm.set_pitch_rate(output_pitch)

class YawPID(object):
    """PID controller for quadcopter Attitude (roll, pitch)

    Parameters
    ----------
    drone_comm: DroneComm instance
        communication object with drone
    yaw_gains: {'kp': kp, 'ki': ki, 'kd': kd}
    """
    def __init__(
            self, drone_comm, yaw_gains, out_yaw_limit=1.0,
            ):
        self.yaw_controller = PID(
            yaw_gains['kp'], yaw_gains['ki'], yaw_gains['kd'],
            lambda:yaw0, drone.get_yaw, drone.get_dyaw,
            _min_angle, out_yaw_limit)

    def step(self):
        output_yaw = self.yaw_controller.step()
        self.drone_comm.set_yaw_rate(output_yaw)
