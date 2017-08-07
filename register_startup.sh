#!/bin/bash

# Registers startup.py to be run at boot time so that pwm signals are valid
# Requires sudo

STARTUP_NAME=i2c_pwm_startup.py
STARTUP_PATH=/etc/init.d/$STARTUP_NAME

if [ -e "$STARTUP_PATH" ]; then
    sudo rm $STARTUP_PATH
fi
sudo ln -s $PWD/startup.py $STARTUP_PATH
sudo chmod 755 startup.py
sudo update-rc.d $STARTUP_NAME defaults

# Registers pigpiod (pigpio daemon) to be started at boot time so that the
# pigpio library works by default without having to manually initialize the
# daemon

PIGPIOD_STARTUP_NAME=pigpiod
PIGPIOD_STARTUP_PATH=/etc/init.d/$PIGPIOD_STARTUP_NAME
sudo ln -s $PWD/pigpiod $PIGPIOD_STARTUP_PATH
sudo chmod 755 pigpiod
sudo update-rc.d $PIGPIOD_STARTUP_NAME defaults
sudo update-rc.d $PIGPIOD_STARTUP_NAME enable
