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

# Commands
# We send roll pitch yaw rates to the flight control board
REDIS_CMD_DROLL = "CMD_DROLL"
REDIS_CMD_DPITCH = "CMD_DPITCH"
REDIS_CMD_DYAW = "CMD_DYAW"

# Sonar data
# downwards facing sonar provides height off ground
# front/back facing sonars provide obstacle avoidance
REDIS_SONAR_DOWN = "SONAR_DOWN"
REDIS_SONAR_FRONT = "SONAR_FRONT"
REDIS_SONAR_BACK = "SONAR_BACK"

class DBRedis(object):
    """Class to set and get redis data

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
        self.rdb.flush()
        # IMU data
        self.rdb.set(REDIS_IMU_DROLL, 0)
        self.rdb.set(REDIS_IMU_DPITCH, 0)
        self.rdb.set(REDIS_IMU_DYAW, 0)
        self.rdb.set(REDIS_IMU_DDX, 0)
        self.rdb.set(REDIS_IMU_DDY, 0)
        self.rdb.set(REDIS_IMU_DDZ, 0)
        # Commands
        self.rdb.set(REDIS_CMD_DROLL, 0)
        self.rdb.set(REDIS_CMD_DPITCH, 0)
        self.rdb.set(REDIS_CMD_DYAW, 0)
        # Sonar data
        self.rdb.set(REDIS_SONAR_DOWN, 0)
        self.rdb.set(REDIS_SONAR_FRONT, 0)
        self.rdb.set(REDIS_SONAR_BACK, 0)

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

    def get_cmd(self):
        """Get the Command data

        Returns the list of command data
            [droll, dpitch, dyaw]
        """
        self.rdb_pipe.get(REDIS_CMD_DROLL)
        self.rdb_pipe.get(REDIS_CMD_DPITCH)
        self.rdb_pipe.get(REDIS_CMD_DYAW)
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
        """Set the IMU data

        Inputs
        ------
        imu_data : list of ints
            [droll, dpitch, dyaw, ddx, ddy, ddz]
        """
        self.rdb_pipe.set(REDIS_IMU_DROLL, imu_data[0])
        self.rdb_pipe.set(REDIS_IMU_DPITCH, imu_data[1])
        self.rdb_pipe.set(REDIS_IMU_DYAW, imu_data[2])
        self.rdb_pipe.set(REDIS_IMU_DDX, imu_data[3])
        self.rdb_pipe.set(REDIS_IMU_DDY, imu_data[4])
        self.rdb_pipe.set(REDIS_IMU_DDZ, imu_data[5])
        self.rdb_pipe.execute()

    def set_cmd(self, cmd_data):
        """Set the Command data

        Inputs
        ------
        cmd_data : list of ints
            [droll, dpitch, dyaw]
        """
        self.rdb_pipe.set(REDIS_CMD_DROLL, cmd_data[0])
        self.rdb_pipe.set(REDIS_CMD_DPITCH, cmd_data[1])
        self.rdb_pipe.set(REDIS_CMD_DYAW, cmd_data[2])
        self.rdb_pipe.execute()

    def set_sonar(self, sonar_data):
        """Set the Sonar data

        Inputs
        ------
        sonar_data : list of ints
            [down, front, back]
        """
        self.rdb_pipe.set(REDIS_SONAR_DOWN, sonar_data[0])
        self.rdb_pipe.set(REDIS_SONAR_FRONT, sonar_data[1])
        self.rdb_pipe.set(REDIS_SONAR_BACK, sonar_data[2])
        self.rdb_pipe.execute()
