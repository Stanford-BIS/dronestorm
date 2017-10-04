"""This module defines redis keys and provides utility functions

redis is a key-value database for transferring values between modules
running on the Pi. Since any process can access any of the data in the redis
database, we coordinate all of the key names centrally in this module.

Be sure to have previously installed redis with
`sudo apt install redis-server`
and have the redis server up and running
"""
import redis
from dronestorm.comm.rx_util import (
    REMOTE_RX_DROLL_IDX,
    REMOTE_RX_DPITCH_IDX,
    REMOTE_RX_DYAW_IDX,
    REMOTE_RX_THROTTLE_IDX,
    REMOTE_RX_AUX1_IDX,
    REMOTE_RX_AUX2_IDX,
    MSP_THROTTLE_IDX,
    MSP_DROLL_IDX,
    MSP_DPITCH_IDX,
    MSP_DYAW_IDX,
    MSP_AUX1_IDX,
    MSP_AUX2_IDX)

# ATTITUDE data
# Flight control board computes attitude from gyro data
REDIS_ATTITUDE_ROLL = "ATTITUDE_ROLL"
REDIS_ATTITUDE_PITCH = "ATTITUDE_PITCH"
REDIS_ATTITUDE_YAW = "ATTITUDE_YAW"
REDIS_ATTITUDE_CHANNEL = "ATTITUDE"

# IMU data
# gyroscope provides roll pitch yaw rates,
# accelerometer provides x y z accelerations
REDIS_IMU_DROLL = "IMU_DROLL"
REDIS_IMU_DPITCH = "IMU_DPITCH"
REDIS_IMU_DYAW = "IMU_DYAW"
REDIS_IMU_DDX = "IMU_DDX"
REDIS_IMU_DDY = "IMU_DDY"
REDIS_IMU_DDZ = "IMU_DDZ"
REDIS_IMU_CHANNEL = "IMU"

# GPS data
REDIS_GPS_CHANNEL = "GPS"

# State estimation data
# When we want to estimate our state based on the sensor readings
REDIS_STATE_EST_X = "STATE_EST_X"
REDIS_STATE_EST_Y = "STATE_EST_Y"
REDIS_STATE_EST_Z = "STATE_EST_Z"
REDIS_STATE_EST_ROLL = "STATE_EST_ROLL"
REDIS_STATE_EST_PITCH = "STATE_EST_PITCH"
REDIS_STATE_EST_YAW = "STATE_EST_YAW"
REDIS_STATE_EST_DX = "STATE_EST_X"
REDIS_STATE_EST_DY = "STATE_EST_Y"
REDIS_STATE_EST_DZ = "STATE_EST_Z"
REDIS_STATE_EST_OMEGA_X = "STATE_EST_WX"
REDIS_STATE_EST_OMEGA_Y = "STATE_EST_WY"
REDIS_STATE_EST_OMEGA_Z = "STATE_EST_WZ"
REDIS_STATE_EST_CHANNEL = "STATE_EST"

# RX inputs in RC units
# We receive droll dpitch dyaw throttle aux1 aux2 from the rc receiver
REDIS_RX_RC_THROTTLE = "RX_RC_THROTTLE"
REDIS_RX_RC_DROLL = "RX_RC_DROLL"
REDIS_RX_RC_DPITCH = "RX_RC_DPITCH"
REDIS_RX_RC_DYAW = "RX_RC_DYAW"
REDIS_RX_RC_AUX1 = "RX_RC_AUX1"
REDIS_RX_RC_AUX2 = "RX_RC_AUX2"
REDIS_RX_RC_CHANNEL = "RX_RC"

# RX inputs normalized
# We receive droll dpitch dyaw throttle aux1 aux2 from the rc receiver
# droll, dpitch, dyaw normalized to [-1, 1]
# throttle, AUX1, AUX2 normalized to [0, 1]
REDIS_RX_THROTTLE = "RX_THROTTLE"
REDIS_RX_DROLL = "RX_DROLL"
REDIS_RX_DPITCH = "RX_DPITCH"
REDIS_RX_DYAW = "RX_DYAW"
REDIS_RX_AUX1 = "RX_AUX1"
REDIS_RX_AUX2 = "RX_AUX2"
REDIS_RX_CHANNEL = "RX"

# Commands normalized
# We send roll pitch yaw aux1 aux2 to the flight control board
REDIS_CMD_RC_DROLL = "CMD_RC_DROLL"
REDIS_CMD_RC_DPITCH = "CMD_RC_DPITCH"
REDIS_CMD_RC_DYAW = "CMD_RC_DYAW"
REDIS_CMD_RC_THROTTLE = "CMD_RC_THROTTLE"
REDIS_CMD_RC_AUX1 = "CMD_RC_AUX1"
REDIS_CMD_RC_AUX2 = "CMD_RC_AUX2"
REDIS_CMD_RC_CHANNEL = "CMD_RC"

# Commands normalized
# We send roll pitch yaw aux1 aux2 to the flight control board
REDIS_CMD_DROLL = "CMD_DROLL"
REDIS_CMD_DPITCH = "CMD_DPITCH"
REDIS_CMD_DYAW = "CMD_DYAW"
REDIS_CMD_THROTTLE = "CMD_THROTTLE"
REDIS_CMD_AUX1 = "CMD_AUX1"
REDIS_CMD_AUX2 = "CMD_AUX2"
REDIS_CMD_CHANNEL = "CMD"

# Sonar data
# downwards facing sonar provides height off ground
# front/back facing sonars provide obstacle avoidance
REDIS_SONAR_DOWN = "SONAR_DOWN"
REDIS_SONAR_FRONT = "SONAR_FRONT"
REDIS_SONAR_BACK = "SONAR_BACK"
REDIS_SONAR_CHANNEL = "SONAR"

class DBRedis(object):
    """Class to set and get redis data

    For IMU, CMD, and SONAR data, provides pipelined functions
    for faster execution and publisher/subscriber notifications of new data

    Parameters
    ----------
    host : string (default "localhost"
        host name or ip address of redis database
    port : int
        port number of redis database
    """
    def __init__(self, host="localhost", port=6379):
        self.rdb = redis.StrictRedis(host=host, port=port, db=0)
        self.rdb_pipe = self.rdb.pipeline()

    def reset_db(self):
        """Reset the redis database

        Beware that the redis database is accessible by any process,
        so it's good practice to designate a single process to control reset
        """
        # IMU data
        self.rdb.set(REDIS_IMU_DROLL, 0)
        self.rdb.set(REDIS_IMU_DPITCH, 0)
        self.rdb.set(REDIS_IMU_DYAW, 0)
        self.rdb.set(REDIS_IMU_DDX, 0)
        self.rdb.set(REDIS_IMU_DDY, 0)
        self.rdb.set(REDIS_IMU_DDZ, 0)
        # RX data
        self.rdb.set(REDIS_RX_THROTTLE, 0)
        self.rdb.set(REDIS_RX_DROLL, 0)
        self.rdb.set(REDIS_RX_DPITCH, 0)
        self.rdb.set(REDIS_RX_DYAW, 0)
        self.rdb.set(REDIS_RX_AUX1, 0)
        self.rdb.set(REDIS_RX_AUX2, 0)
        self.rdb.set(REDIS_RX_RC_THROTTLE, 0)
        self.rdb.set(REDIS_RX_RC_DROLL, 0)
        self.rdb.set(REDIS_RX_RC_DPITCH, 0)
        self.rdb.set(REDIS_RX_RC_DYAW, 0)
        self.rdb.set(REDIS_RX_RC_AUX1, 0)
        self.rdb.set(REDIS_RX_RC_AUX2, 0)
        # Commands
        self.rdb.set(REDIS_CMD_THROTTLE, 0)
        self.rdb.set(REDIS_CMD_DROLL, 0)
        self.rdb.set(REDIS_CMD_DPITCH, 0)
        self.rdb.set(REDIS_CMD_DYAW, 0)
        self.rdb.set(REDIS_CMD_AUX1, 0)
        self.rdb.set(REDIS_CMD_AUX2, 0)
        self.rdb.set(REDIS_CMD_RC_THROTTLE, 0)
        self.rdb.set(REDIS_CMD_RC_DROLL, 0)
        self.rdb.set(REDIS_CMD_RC_DPITCH, 0)
        self.rdb.set(REDIS_CMD_RC_DYAW, 0)
        self.rdb.set(REDIS_CMD_RC_AUX1, 0)
        self.rdb.set(REDIS_CMD_RC_AUX2, 0)
        # Sonar data
        self.rdb.set(REDIS_SONAR_DOWN, 0)
        self.rdb.set(REDIS_SONAR_FRONT, 0)
        self.rdb.set(REDIS_SONAR_BACK, 0)
        # State estimate
        self.rdb.set(REDIS_STATE_EST_X, 0)
        self.rdb.set(REDIS_STATE_EST_Y, 0)
        self.rdb.set(REDIS_STATE_EST_Z, 0)
        self.rdb.set(REDIS_STATE_EST_ROLL, 0)
        self.rdb.set(REDIS_STATE_EST_PITCH, 0)
        self.rdb.set(REDIS_STATE_EST_YAW, 0)
        self.rdb.set(REDIS_STATE_EST_DX, 0)
        self.rdb.set(REDIS_STATE_EST_DY, 0)
        self.rdb.set(REDIS_STATE_EST_DZ, 0)
        self.rdb.set(REDIS_STATE_EST_OMEGA_X, 0)
        self.rdb.set(REDIS_STATE_EST_OMEGA_Y, 0)
        self.rdb.set(REDIS_STATE_EST_OMEGA_Z, 0)

    def subscribe(self, chans):
        """Create a pubsub object

        Inputs
        ------
        chans : string or list of strings
            channel(s) to subscribe to
        """
        pubsub = self.rdb.pubsub(ignore_subscribe_messages=True)
        if not isinstance(chans, (list, tuple)):
            chans = [chans]
        for chan in chans:
            pubsub.subscribe(chan)
        return pubsub

###############################################################################
# getter utilities ############################################################
###############################################################################

def get_attitude(db_redis):
    """Get the attitude data

    Returns the list of attitude data
        [roll, pitch, yaw]
    """
    db_redis.rdb_pipe.get(REDIS_ATTITUDE_ROLL)
    db_redis.rdb_pipe.get(REDIS_ATTITUDE_PITCH)
    db_redis.rdb_pipe.get(REDIS_ATTITUDE_YAW)
    imu_dat = db_redis.rdb_pipe.execute()
    return list(map(float, imu_dat))

def get_cmd(db_redis):
    """Get the Command data

    Returns the list of command data
        [throttle, droll, dpitch, dyaw, aux1, aux2]
    """
    db_redis.rdb_pipe.get(REDIS_CMD_THROTTLE)
    db_redis.rdb_pipe.get(REDIS_CMD_DROLL)
    db_redis.rdb_pipe.get(REDIS_CMD_DPITCH)
    db_redis.rdb_pipe.get(REDIS_CMD_DYAW)
    db_redis.rdb_pipe.get(REDIS_CMD_AUX1)
    db_redis.rdb_pipe.get(REDIS_CMD_AUX2)
    cmd_data = db_redis.rdb_pipe.execute()
    return list(map(float, cmd_data))

def get_imu(db_redis):
    """Get the IMU data

    Returns the list of imu data
        [droll, dpitch, dyaw, ddx, ddy, ddz]
    """
    db_redis.rdb_pipe.get(REDIS_IMU_DROLL)
    db_redis.rdb_pipe.get(REDIS_IMU_DPITCH)
    db_redis.rdb_pipe.get(REDIS_IMU_DYAW)
    db_redis.rdb_pipe.get(REDIS_IMU_DDX)
    db_redis.rdb_pipe.get(REDIS_IMU_DDY)
    db_redis.rdb_pipe.get(REDIS_IMU_DDZ)
    imu_dat = db_redis.rdb_pipe.execute()
    return list(map(int, imu_dat))

def get_key(db_redis, key, value_fun=None):
    """Generic redis get command
    
    Inputs
    ------
    key: string
        redis database key
    value_fun: callable or None
        if not None, will be called on value returned by redis
        redis returns the value as a string
        for example, pass in int to cast the value as an int
    """
    value = db_redis.rdb.get(key)
    if value_fun is not None:
        value = value_fun(value)
    return value

def get_rx(db_redis):
    """Get the receiver data

    Returns the list of receiver data
        [throttle, droll, dpitch, dyaw, aux1, aux2]
    """
    db_redis.rdb_pipe.get(REDIS_RX_THROTTLE)
    db_redis.rdb_pipe.get(REDIS_RX_DROLL)
    db_redis.rdb_pipe.get(REDIS_RX_DPITCH)
    db_redis.rdb_pipe.get(REDIS_RX_DYAW)
    db_redis.rdb_pipe.get(REDIS_RX_AUX1)
    db_redis.rdb_pipe.get(REDIS_RX_AUX2)
    rx_data = db_redis.rdb_pipe.execute()
    return list(map(float, rx_data))

def get_rx_rc(db_redis):
    """Get the receiver data in RC units

    Returns the list of receiver data in RC units
        [throttle, droll, dpitch, dyaw, aux1, aux2]
    """
    db_redis.rdb_pipe.get(REDIS_RX_RC_THROTTLE)
    db_redis.rdb_pipe.get(REDIS_RX_RC_DROLL)
    db_redis.rdb_pipe.get(REDIS_RX_RC_DPITCH)
    db_redis.rdb_pipe.get(REDIS_RX_RC_DYAW)
    db_redis.rdb_pipe.get(REDIS_RX_RC_AUX1)
    db_redis.rdb_pipe.get(REDIS_RX_RC_AUX2)
    rx_data = db_redis.rdb_pipe.execute()
    return list(map(int, rx_data))

def get_sonar(db_redis):
    """Get the sonar data

    Returns the list of sonar data
        [down, front, back]
    """
    db_redis.rdb_pipe.get(REDIS_SONAR_DOWN)
    db_redis.rdb_pipe.get(REDIS_SONAR_FRONT)
    db_redis.rdb_pipe.get(REDIS_SONAR_BACK)
    sonar_data = db_redis.rdb_pipe.execute()
    return list(map(int, sonar_data))

def get_state_estimate(db_redis):
    """get the state estimate data

    Returns list of state estimates
        [x, y, z, roll, pitch, yaw, dx, dy, dz, omega_x, omega_y, omega_z]
    """
    db_redis.rdb.pipe.get(REDIS_STATE_EST_X)
    db_redis.rdb.pipe.get(REDIS_STATE_EST_Y)
    db_redis.rdb.pipe.get(REDIS_STATE_EST_Z)
    db_redis.rdb.pipe.get(REDIS_STATE_EST_ROLL)
    db_redis.rdb.pipe.get(REDIS_STATE_EST_PITCH)
    db_redis.rdb.pipe.get(REDIS_STATE_EST_YAW)
    db_redis.rdb.pipe.get(REDIS_STATE_EST_DX)
    db_redis.rdb.pipe.get(REDIS_STATE_EST_DY)
    db_redis.rdb.pipe.get(REDIS_STATE_EST_DZ)
    db_redis.rdb.pipe.get(REDIS_STATE_EST_OMEGA_X)
    db_redis.rdb.pipe.get(REDIS_STATE_EST_OMEGA_Y)
    db_redis.rdb.pipe.get(REDIS_STATE_EST_OMEGA_Z)
    return list(map(float(db_redis.rdb_pipe.execute())))

###############################################################################
# setter utilities ############################################################
###############################################################################

def set_attitude(db_redis, attitude_data):
    """Set the attitude data and notify REDIS_IMU subscribers

    Inputs
    ------
    attitude_data : list of ints
        [droll, dpitch, dyaw, ddx, ddy, ddz]
    """
    assert len(attitude_data) == 3, "attitude_data must be list of 3 ints"
    db_redis.rdb_pipe.set(REDIS_ATTITUDE_ROLL, attitude_data[0])
    db_redis.rdb_pipe.set(REDIS_ATTITUDE_PITCH, attitude_data[1])
    db_redis.rdb_pipe.set(REDIS_ATTITUDE_YAW, attitude_data[2])
    db_redis.rdb_pipe.execute()
    db_redis.rdb.publish(REDIS_ATTITUDE_CHANNEL, 1)

def set_cmd(db_redis, cmd_data):
    """Set the Command data in normaed units and notify REDIS_CMD subscribers

    Inputs
    ------
    cmd_data : list of float
        Follows the MultiWii Serial Protocol channel indexing
        [throttle, droll, dpitch, dyaw, aux1, aux2]
    """
    assert len(cmd_data) == 6, "cmd_data must be list of 6 ints"
    db_redis.rdb_pipe.set(REDIS_CMD_THROTTLE, cmd_data[MSP_THROTTLE_IDX])
    db_redis.rdb_pipe.set(REDIS_CMD_DROLL, cmd_data[MSP_DROLL_IDX])
    db_redis.rdb_pipe.set(REDIS_CMD_DPITCH, cmd_data[MSP_DPITCH_IDX])
    db_redis.rdb_pipe.set(REDIS_CMD_DYAW, cmd_data[MSP_DYAW_IDX])
    db_redis.rdb_pipe.set(REDIS_CMD_AUX1, cmd_data[MSP_AUX1_IDX])
    db_redis.rdb_pipe.set(REDIS_CMD_AUX2, cmd_data[MSP_AUX2_IDX])
    db_redis.rdb_pipe.execute()
    db_redis.rdb.publish(REDIS_CMD_CHANNEL, 1)

def set_cmd_rc(db_redis, cmd_rc_data):
    """Set the Command data in RC units and notify REDIS_CMD_RC subscribers

    Inputs
    ------
    cmd_data : list of ints
        [throttle, droll, dpitch, dyaw, aux1, aux2]
    """
    assert len(cmd_rc_data) == 6, "cmd_data must be list of 6 ints"
    db_redis.rdb_pipe.set(REDIS_CMD_RC_THROTTLE, cmd_rc_data[MSP_THROTTLE_IDX])
    db_redis.rdb_pipe.set(REDIS_CMD_RC_DROLL, cmd_rc_data[MSP_DROLL_IDX])
    db_redis.rdb_pipe.set(REDIS_CMD_RC_DPITCH, cmd_rc_data[MSP_DPITCH_IDX])
    db_redis.rdb_pipe.set(REDIS_CMD_RC_DYAW, cmd_rc_data[MSP_DYAW_IDX])
    db_redis.rdb_pipe.set(REDIS_CMD_RC_AUX1, cmd_rc_data[MSP_AUX1_IDX])
    db_redis.rdb_pipe.set(REDIS_CMD_RC_AUX2, cmd_rc_data[MSP_AUX2_IDX])
    db_redis.rdb_pipe.execute()
    db_redis.rdb.publish(REDIS_CMD_RC_CHANNEL, 1)

def set_imu(db_redis, imu_data):
    """Set the IMU data and notify REDIS_IMU subscribers

    Inputs
    ------
    imu_data : list of ints
        [droll, dpitch, dyaw, ddx, ddy, ddz]
    """
    assert len(imu_data) == 6, "imu_data must be list of 6 ints"
    db_redis.rdb_pipe.set(REDIS_IMU_DDX, imu_data[0])
    db_redis.rdb_pipe.set(REDIS_IMU_DDY, imu_data[1])
    db_redis.rdb_pipe.set(REDIS_IMU_DDZ, imu_data[2])
    db_redis.rdb_pipe.set(REDIS_IMU_DROLL, imu_data[3])
    db_redis.rdb_pipe.set(REDIS_IMU_DPITCH, imu_data[4])
    db_redis.rdb_pipe.set(REDIS_IMU_DYAW, imu_data[5])
    db_redis.rdb_pipe.execute()
    db_redis.rdb.publish(REDIS_IMU_CHANNEL, 1)

def set_rx(db_redis, rx_data):
    """Set the receiver and notify REDIS_RX subscribers

    Inputs
    ------
    cmd_data : list of floats
        Follows the Spektrum Remote Receiver channel indexing
        [throttle, droll, dpitch, dyaw, AUX1, AUX2]
    """
    assert len(rx_data) == 6, "rx_data must be list of 6 floats"
    db_redis.rdb_pipe.set(REDIS_RX_THROTTLE, rx_data[REMOTE_RX_THROTTLE_IDX])
    db_redis.rdb_pipe.set(REDIS_RX_DROLL, rx_data[REMOTE_RX_DROLL_IDX])
    db_redis.rdb_pipe.set(REDIS_RX_DPITCH, rx_data[REMOTE_RX_DPITCH_IDX])
    db_redis.rdb_pipe.set(REDIS_RX_DYAW, rx_data[REMOTE_RX_DYAW_IDX])
    db_redis.rdb_pipe.set(REDIS_RX_AUX1, rx_data[REMOTE_RX_AUX1_IDX])
    db_redis.rdb_pipe.set(REDIS_RX_AUX2, rx_data[REMOTE_RX_AUX2_IDX])
    db_redis.rdb_pipe.execute()
    db_redis.rdb.publish(REDIS_RX_CHANNEL, 1)

def set_rx_rc(db_redis, rx_rc_data):
    """Set the receiver data in RC units and notify REDIS_RX_RC subscribers

    Inputs
    ------
    rx_rc_data : list of ints
        Follows the Spektrum Remote Receiver channel indexing
        [throttle, droll, dpitch, dyaw, AUX1, AUX2]
    """
    assert len(rx_rc_data) == 6, "rx_rc_data must be list of 6 ints"
    db_redis.rdb_pipe.set(REDIS_RX_RC_DROLL, rx_rc_data[REMOTE_RX_DROLL_IDX])
    db_redis.rdb_pipe.set(REDIS_RX_RC_DPITCH, rx_rc_data[REMOTE_RX_DPITCH_IDX])
    db_redis.rdb_pipe.set(REDIS_RX_RC_DYAW, rx_rc_data[REMOTE_RX_DYAW_IDX])
    db_redis.rdb_pipe.set(REDIS_RX_RC_THROTTLE, rx_rc_data[REMOTE_RX_THROTTLE_IDX])
    db_redis.rdb_pipe.set(REDIS_RX_RC_AUX1, rx_rc_data[REMOTE_RX_AUX1_IDX])
    db_redis.rdb_pipe.set(REDIS_RX_RC_AUX2, rx_rc_data[REMOTE_RX_AUX2_IDX])
    db_redis.rdb_pipe.execute()
    db_redis.rdb.publish(REDIS_RX_RC_CHANNEL, 1)

def set_sonar(db_redis, sonar_data):
    """Set the sonar data and notify REDIS_SONAR subscribers of new data

    Inputs
    ------
    sonar_data : list of ints
        [down, front, back]
    """
    db_redis.rdb_pipe.set(REDIS_SONAR_DOWN, sonar_data[0])
    db_redis.rdb_pipe.set(REDIS_SONAR_FRONT, sonar_data[1])
    db_redis.rdb_pipe.set(REDIS_SONAR_BACK, sonar_data[2])
    db_redis.rdb_pipe.execute()
    db_redis.rdb.publish(REDIS_SONAR_CHANNEL, 1)

def set_state_estimate(db_redis, state_est):
    """Set the state estimate data and notify REDIS_STATE_EST subscribers of new data

    Inputs
    ------
    state_est : list of floats
        [x, y, z, roll, pitch, yaw, dx, dy, dz, omega_x, omega_y, omega_z]
    """
    db_redis.rdb.pipe.set(REDIS_STATE_EST_X, state_est[0])
    db_redis.rdb.pipe.set(REDIS_STATE_EST_Y, state_est[1])
    db_redis.rdb.pipe.set(REDIS_STATE_EST_Z, state_est[2])
    db_redis.rdb.pipe.set(REDIS_STATE_EST_ROLL, state_est[3])
    db_redis.rdb.pipe.set(REDIS_STATE_EST_PITCH, state_est[4])
    db_redis.rdb.pipe.set(REDIS_STATE_EST_YAW, state_est[5])
    db_redis.rdb.pipe.set(REDIS_STATE_EST_DX, state_est[6])
    db_redis.rdb.pipe.set(REDIS_STATE_EST_DY, state_est[7])
    db_redis.rdb.pipe.set(REDIS_STATE_EST_DZ, state_est[8])
    db_redis.rdb.pipe.set(REDIS_STATE_EST_OMEGA_X, state_est[9])
    db_redis.rdb.pipe.set(REDIS_STATE_EST_OMEGA_Y, state_est[10])
    db_redis.rdb.pipe.set(REDIS_STATE_EST_OMEGA_Z, state_est[11])
    db_redis.rdb_pipe.execute()
    db_redis.rdb.publish(REDIS_STATE_EST_CHANNEL, 1)
