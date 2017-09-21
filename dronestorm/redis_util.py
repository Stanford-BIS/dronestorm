"""This module defines redis keys and provides utility functions

redis is a key-value database for transferring values between modules
running on the Pi. Since any process can access any of the data in the redis
database, we coordinate all of the key names centrally in this module.

Be sure to have previously installed redis with
`sudo apt install redis-server`
and have the redis server up and running
"""
import redis

# ATTITUDE data
# Flight control board computes attitude from gyro data
REDIS_ATTITUDE_ROLL = "ATTITUDE_ROLL"
REDIS_ATTITUDE_PITCH = "ATTITUDE_PITCH"
REDIS_ATTITUDE_YAW = "ATTITUDE_YAW"
REDIS_ATTITUDE_CHANNEL = "ATTITUDE"

# IMU data
# gyroscope provides roll pitch yaw rates,
# accelerometer provides x y z accelerations
REDIS_IMU_DROLL = "IMU_ROLL"
REDIS_IMU_DPITCH = "IMU_PITCH"
REDIS_IMU_DYAW = "IMU_YAW"
REDIS_IMU_DDX = "IMU_X"
REDIS_IMU_DDY = "IMU_Y"
REDIS_IMU_DDZ = "IMU_Z"
REDIS_IMU_CHANNEL = "IMU"

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

# Commands
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
        self.rdb.set(REDIS_RX_DROLL, 0)
        self.rdb.set(REDIS_RX_DPITCH, 0)
        self.rdb.set(REDIS_RX_DYAW, 0)
        self.rdb.set(REDIS_RX_THROTTLE, 0)
        self.rdb.set(REDIS_RX_AUX1, 0)
        self.rdb.set(REDIS_RX_AUX2, 0)
        # Commands
        self.rdb.set(REDIS_CMD_DROLL, 0)
        self.rdb.set(REDIS_CMD_DPITCH, 0)
        self.rdb.set(REDIS_CMD_DYAW, 0)
        self.rdb.set(REDIS_CMD_THROTTLE, 0)
        self.rdb.set(REDIS_CMD_AUX1, 0)
        self.rdb.set(REDIS_CMD_AUX2, 0)
        # Sonar data
        self.rdb.set(REDIS_SONAR_DOWN, 0)
        self.rdb.set(REDIS_SONAR_FRONT, 0)
        self.rdb.set(REDIS_SONAR_BACK, 0)

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
        [droll, dpitch, dyaw, ddx, ddy, ddz]
    """
    db_redis.rdb_pipe.get(REDIS_ATTITUDE_ROLL)
    db_redis.rdb_pipe.get(REDIS_ATTITUDE_PITCH)
    db_redis.rdb_pipe.get(REDIS_ATTITUDE_YAW)
    imu_dat = db_redis.rdb_pipe.execute()
    return list(map(int, imu_dat))

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

def get_rx(db_redis):
    """Get the receiver data

    Returns the list of receiver data
        [droll, dpitch, dyaw, throttle, aux1, aux2]
    """
    db_redis.rdb_pipe.get(REDIS_RX_DROLL)
    db_redis.rdb_pipe.get(REDIS_RX_DPITCH)
    db_redis.rdb_pipe.get(REDIS_RX_DYAW)
    db_redis.rdb_pipe.get(REDIS_RX_THROTTLE)
    db_redis.rdb_pipe.get(REDIS_RX_AUX1)
    db_redis.rdb_pipe.get(REDIS_RX_AUX2)
    rx_data = db_redis.rdb_pipe.execute()
    return list(map(float, rx_data))

def get_rx_rc(db_redis):
    """Get the receiver data in RC units

    Returns the list of receiver data in RC units
        [droll, dpitch, dyaw, throttle, aux1, aux2]
    """
    db_redis.rdb_pipe.get(REDIS_RX_RC_DROLL)
    db_redis.rdb_pipe.get(REDIS_RX_RC_DPITCH)
    db_redis.rdb_pipe.get(REDIS_RX_RC_DYAW)
    db_redis.rdb_pipe.get(REDIS_RX_RC_THROTTLE)
    db_redis.rdb_pipe.get(REDIS_RX_RC_AUX1)
    db_redis.rdb_pipe.get(REDIS_RX_RC_AUX2)
    rx_data = db_redis.rdb_pipe.execute()
    return list(map(int, rx_data))

def get_cmd(db_redis):
    """Get the Command data

    Returns the list of command data
        [droll, dpitch, dyaw, throttle, aux1, aux2]
    """
    db_redis.rdb_pipe.get(REDIS_CMD_DROLL)
    db_redis.rdb_pipe.get(REDIS_CMD_DPITCH)
    db_redis.rdb_pipe.get(REDIS_CMD_DYAW)
    db_redis.rdb_pipe.get(REDIS_CMD_THROTTLE)
    db_redis.rdb_pipe.get(REDIS_CMD_AUX1)
    db_redis.rdb_pipe.get(REDIS_CMD_AUX2)
    cmd_data = db_redis.rdb_pipe.execute()
    return list(map(int, cmd_data))

def get_sonar(db_redis):
    """Get the Sonar data

    Returns the list of sonar data
        [down, front, back]
    """
    db_redis.rdb_pipe.get(REDIS_SONAR_DOWN)
    db_redis.rdb_pipe.get(REDIS_SONAR_FRONT)
    db_redis.rdb_pipe.get(REDIS_SONAR_BACK)
    sonar_data = db_redis.rdb_pipe.execute()
    return list(map(int, sonar_data))

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

def set_imu(db_redis, imu_data):
    """Set the IMU data and notify REDIS_IMU subscribers

    Inputs
    ------
    imu_data : list of ints
        [droll, dpitch, dyaw, ddx, ddy, ddz]
    """
    assert len(imu_data) == 6, "imu_data must be list of 6 ints"
    db_redis.rdb_pipe.set(REDIS_IMU_DROLL, imu_data[0])
    db_redis.rdb_pipe.set(REDIS_IMU_DPITCH, imu_data[1])
    db_redis.rdb_pipe.set(REDIS_IMU_DYAW, imu_data[2])
    db_redis.rdb_pipe.set(REDIS_IMU_DDX, imu_data[3])
    db_redis.rdb_pipe.set(REDIS_IMU_DDY, imu_data[4])
    db_redis.rdb_pipe.set(REDIS_IMU_DDZ, imu_data[5])
    db_redis.rdb_pipe.execute()
    db_redis.rdb.publish(REDIS_IMU_CHANNEL, 1)

def set_rx(db_redis, rx_data):
    """Set the receiver and notify REDIS_RX subscribers

    Inputs
    ------
    cmd_data : list of floats
        [throttle, droll, dpitch, dyaw, AUX1, AUX2]
    """
    assert len(rx_data) == 6, "rx_data must be list of 6 floats"
    db_redis.rdb_pipe.set(REDIS_RX_DROLL, rx_data[0])
    db_redis.rdb_pipe.set(REDIS_RX_DPITCH, rx_data[1])
    db_redis.rdb_pipe.set(REDIS_RX_DYAW, rx_data[2])
    db_redis.rdb_pipe.set(REDIS_RX_THROTTLE, rx_data[3])
    db_redis.rdb_pipe.set(REDIS_RX_AUX1, rx_data[4])
    db_redis.rdb_pipe.set(REDIS_RX_AUX2, rx_data[5])
    db_redis.rdb_pipe.execute()
    db_redis.rdb.publish(REDIS_RX_CHANNEL, 1)

def set_rx_rc(db_redis, rx_rc_data):
    """Set the receiver data in RC units and notify REDIS_RX_RC subscribers

    Inputs
    ------
    rx_rc_data : list of ints
        [throttle, droll, dpitch, dyaw, AUX1, AUX2]
    """
    assert len(rx_rc_data) == 6, "rx_rc_data must be list of 6 ints"
    db_redis.rdb_pipe.set(REDIS_RX_RC_DROLL, rx_rc_data[0])
    db_redis.rdb_pipe.set(REDIS_RX_RC_DPITCH, rx_rc_data[1])
    db_redis.rdb_pipe.set(REDIS_RX_RC_DYAW, rx_rc_data[2])
    db_redis.rdb_pipe.set(REDIS_RX_RC_THROTTLE, rx_rc_data[3])
    db_redis.rdb_pipe.set(REDIS_RX_RC_AUX1, rx_rc_data[4])
    db_redis.rdb_pipe.set(REDIS_RX_RC_AUX2, rx_rc_data[5])
    db_redis.rdb_pipe.execute()
    db_redis.rdb.publish(REDIS_RX_RC_CHANNEL, 1)

def set_cmd(db_redis, cmd_data):
    """Set the Command data and notify REDIS_CMD subscribers of new data

    Inputs
    ------
    cmd_data : list of ints
        [droll, dpitch, dyaw]
    """
    assert len(cmd_data) == 6, "cmd_data must be list of 6 ints"
    db_redis.rdb_pipe.set(REDIS_CMD_DROLL, cmd_data[0])
    db_redis.rdb_pipe.set(REDIS_CMD_DPITCH, cmd_data[1])
    db_redis.rdb_pipe.set(REDIS_CMD_DYAW, cmd_data[2])
    db_redis.rdb_pipe.set(REDIS_CMD_THROTTLE, cmd_data[3])
    db_redis.rdb_pipe.set(REDIS_CMD_AUX1, cmd_data[4])
    db_redis.rdb_pipe.set(REDIS_CMD_AUX2, cmd_data[5])
    db_redis.rdb_pipe.execute()
    db_redis.rdb.publish(REDIS_CMD_CHANNEL, 1)

def set_sonar(db_redis, sonar_data):
    """Set the Sonar data and notify REDIS_SONAR subscribers of new data

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
