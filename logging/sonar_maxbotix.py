"""Examples script to read from a maxbotix 1242 sonar module over i2c"""
from __future__ import print_function
from dronestorm.sense.sonar_maxbotix import SonarMaxbotix

sonar = SonarMaxbotix()

try:
    while True:
        distance = sonar.read()
        print("distance to closest object:%u"%(distance))
except KeyboardInterrupt:
    sonar.close_i2c()
    print("")
