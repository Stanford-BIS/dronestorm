#!/bin/bash

# Registers startup.py to be run at boot time so that pwm signals are valid
# Requires sudo

STARTUP_NAME=i2c_pwm_startup.py
STARTUP_PATH=/etc/init.d/$STARTUP_NAME

if [ -e "$STARTUP_PATH" ]; then
    sudo rm $STARTUP_PATH
fi
sudo ln -s $PWD/startup.py $STARTUP_PATH
sudo chmod 777 startup.py
sudo update-rc.d $STARTUP_NAME defaults
