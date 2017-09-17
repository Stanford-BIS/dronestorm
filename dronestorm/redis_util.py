"""This module defines redis keys and provides utility functions

redis is a key-value database for transferring values between modules
running on the Pi. Since any process can access any of the data in the redis
database, we coordinate all of the key names centrally in this module.

Be sure to have previously installed redis with
`sudo apt install redis-server`
and have the redis server up and running
"""
import redis

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

# RX inputs
# We receive roll pitch yaw throttle aux1 aux2 from the rc receiver
REDIS_RX_DROLL = "RX_DROLL"
REDIS_RX_DPITCH = "RX_DPITCH"
REDIS_RX_DYAW = "RX_DYAW"
REDIS_RX_THROTTLE = "RX_THROTTLE"
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

    def subscribe(self, chan):
        """create a pubsub object subscribed to chan"""
        pubsub = self.rdb.pubsub(ignore_subscribe_messages=True)
        pubsub.subscribe(chan)
        return pubsub

    def get_int(self, key):
        """Get data from the redis database and cast it to an int"""
        return int(self.rdb.get(key))

    def get_float(self, key):
        """Get data from the redis database and cast it to a float"""
        return float(self.rdb.get(key))

    def get_imu(self):
        """Get the IMU data

        Returns the list of imu data
            [droll, dpitch, dyaw, ddx, ddy, ddz]
        """
        self.rdb_pipe.get(REDIS_IMU_DROLL)
        self.rdb_pipe.get(REDIS_IMU_DPITCH)
        self.rdb_pipe.get(REDIS_IMU_DYAW)
        self.rdb_pipe.get(REDIS_IMU_DDX)
        self.rdb_pipe.get(REDIS_IMU_DDY)
        self.rdb_pipe.get(REDIS_IMU_DDZ)
        imu_dat = self.rdb_pipe.execute()
        return list(map(int, imu_dat))

    def get_rx(self):
        """Get the Receiver data

        Returns the list of receiver data
            [droll, dpitch, dyaw, throttle, aux1, aux2]
        """
        self.rdb_pipe.get(REDIS_RX_DROLL)
        self.rdb_pipe.get(REDIS_RX_DPITCH)
        self.rdb_pipe.get(REDIS_RX_DYAW)
        self.rdb_pipe.get(REDIS_RX_THROTTLE)
        self.rdb_pipe.get(REDIS_RX_AUX1)
        self.rdb_pipe.get(REDIS_RX_AUX2)
        rx_data = self.rdb_pipe.execute()
        return list(map(int, rx_data))

    def get_cmd(self):
        """Get the Command data

        Returns the list of command data
            [droll, dpitch, dyaw, throttle, aux1, aux2]
        """
        self.rdb_pipe.get(REDIS_CMD_DROLL)
        self.rdb_pipe.get(REDIS_CMD_DPITCH)
        self.rdb_pipe.get(REDIS_CMD_DYAW)
        self.rdb_pipe.get(REDIS_CMD_THROTTLE)
        self.rdb_pipe.get(REDIS_CMD_AUX1)
        self.rdb_pipe.get(REDIS_CMD_AUX2)
        cmd_data = self.rdb_pipe.execute()
        return list(map(int, cmd_data))

    def get_sonar(self):
        """Get the Sonar data

        Returns the list of sonar data
            [down, front, back]
        """
        self.rdb_pipe.get(REDIS_SONAR_DOWN)
        self.rdb_pipe.get(REDIS_SONAR_FRONT)
        self.rdb_pipe.get(REDIS_SONAR_BACK)
        sonar_data = self.rdb_pipe.execute()
        return list(map(int, sonar_data))

    def set(self, key, value):
        """Set a key-value pair in the redis database"""
        self.rdb.set(key, value)

    def set_imu(self, imu_data):
        """Set the IMU data and notify REDIS_IMU subscribers of new data

        Inputs
        ------
        imu_data : list of ints
            [droll, dpitch, dyaw, ddx, ddy, ddz]
        """
        assert len(imu_data) == 6, "imu_data must be list of 6 ints"
        self.rdb_pipe.set(REDIS_IMU_DROLL, imu_data[0])
        self.rdb_pipe.set(REDIS_IMU_DPITCH, imu_data[1])
        self.rdb_pipe.set(REDIS_IMU_DYAW, imu_data[2])
        self.rdb_pipe.set(REDIS_IMU_DDX, imu_data[3])
        self.rdb_pipe.set(REDIS_IMU_DDY, imu_data[4])
        self.rdb_pipe.set(REDIS_IMU_DDZ, imu_data[5])
        self.rdb_pipe.execute()
        self.rdb.publish(REDIS_IMU_CHANNEL, 1)

    def set_rx(self, rx_data):
        """Set the RX data and notify REDIS_RX subscribers of new data

        Inputs
        ------
        cmd_data : list of ints
            [droll, dpitch, dyaw]
        """
        assert len(rx_data) == 6, "rx_data must be list of 6 ints"
        self.rdb_pipe.set(REDIS_RX_DROLL, rx_data[0])
        self.rdb_pipe.set(REDIS_RX_DPITCH, rx_data[1])
        self.rdb_pipe.set(REDIS_RX_DYAW, rx_data[2])
        self.rdb_pipe.set(REDIS_RX_THROTTLE, rx_data[3])
        self.rdb_pipe.set(REDIS_RX_AUX1, rx_data[4])
        self.rdb_pipe.set(REDIS_RX_AUX2, rx_data[5])
        self.rdb_pipe.execute()
        self.rdb.publish(REDIS_RX_CHANNEL, 1)

    def set_cmd(self, cmd_data):
        """Set the Command data and notify REDIS_CMD subscribers of new data

        Inputs
        ------
        cmd_data : list of ints
            [droll, dpitch, dyaw]
        """
        assert len(cmd_data) == 6, "cmd_data must be list of 6 ints"
        self.rdb_pipe.set(REDIS_CMD_DROLL, cmd_data[0])
        self.rdb_pipe.set(REDIS_CMD_DPITCH, cmd_data[1])
        self.rdb_pipe.set(REDIS_CMD_DYAW, cmd_data[2])
        self.rdb_pipe.set(REDIS_CMD_THROTTLE, cmd_data[3])
        self.rdb_pipe.set(REDIS_CMD_AUX1, cmd_data[4])
        self.rdb_pipe.set(REDIS_CMD_AUX2, cmd_data[5])
        self.rdb_pipe.execute()
        self.rdb.publish(REDIS_CMD_CHANNEL, 1)

    def set_sonar(self, sonar_data):
        """Set the Sonar data and notify REDIS_SONAR subscribers of new data

        Inputs
        ------
        sonar_data : list of ints
            [down, front, back]
        """
        self.rdb_pipe.set(REDIS_SONAR_DOWN, sonar_data[0])
        self.rdb_pipe.set(REDIS_SONAR_FRONT, sonar_data[1])
        self.rdb_pipe.set(REDIS_SONAR_BACK, sonar_data[2])
        self.rdb_pipe.execute()
        self.rdb.publish(REDIS_SONAR_CHANNEL, 1)
