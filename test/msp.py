# test the MultiWii Serial Protocol communication
import dronestorm.comm as comm
from dronestorm.comm import MultiWii, msp
from dronestorm.comm.msp.msp_types import (
    RX_SERIAL_SPEKTRUM2048, RX_MSP,
    FIRMWARE_BF)
import time
from pprint import pprint

mw = MultiWii(
    port="/dev/ttyACM0",
    rx_protocol=RX_MSP,
    firmware=FIRMWARE_BF)

print("getting rx config")
pprint(msp.get_rx_config(mw))
print()

print("getting status")
pprint(msp.get_status(mw))
print()
time.sleep(0.1)

print("getting motor")
print(msp.get_motor(mw))
for n in range(10):
    for motor_value in [100*i for i in range(10,21)]:
        print("setting motor_value to %d"%motor_value)
        msp.set_motor(mw, [motor_value for i in range(4)])
        print("checking that motor values are set to %d"%motor_value)
        print(msp.get_motor(mw))
        print()

print("getting rc")
print(msp.get_rc(mw))
for n in range(10):
    for rc_value in [100*i for i in range(10,21)]:
        print("setting rc to %d"%rc_value)
        msp.set_rc(mw, [rc_value for i in range(6)])
        print("checking that rc set to %d"%rc_value)
        print(msp.get_rc(mw))
    print()
    
print("getting imu")
for n in range(10):
    print(msp.get_raw_imu(mw))

mw.close_serial()
