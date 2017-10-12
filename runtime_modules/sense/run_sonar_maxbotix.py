"""communicates with the downwards facing maxbotix sonar during runtime

Reads distance to closest object data from the sonar
Wriites distance to closest object data to the redis database
"""
from __future__ import print_function
import os
import sys
from dronestorm.sense.sonar_maxbotix import SonarMaxbotix
from dronestorm.comm.redis_util import DBRedis, set_sonar

def run_sonar_maxbotix():
    print(os.path.basename(__file__))
    db_redis = DBRedis()

    sonar = SonarMaxbotix()
    try:
        print("Running Maxbotix sonar...Ctrl-c to stop")
        print("Down Distance")
        while True:
            down_distance = sonar.read()
            set_sonar(db_redis, [0, down_distance, 0])
            sys.stdout.write("%3u\r"%(down_distance))
            sys.stdout.flush()
    except KeyboardInterrupt:
        print("")
        print("Closing I2C port to Maxbotix sonar")
        sonar.close_i2c()

if __name__ == "__main__":
    run_sonar_maxbotix()
