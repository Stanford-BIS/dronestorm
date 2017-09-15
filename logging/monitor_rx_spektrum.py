"""Monitor Spektrum remote receiver data"""
import sys
from dronestorm.comm import SpektrumRemoteReceiver

print("Throttle     Roll    Pitch      Yaw     AUX1     AUX2")

rrx = SpektrumRemoteReceiver()
try:
    rrx.align_serial()
    while True:
        rc_data = rrx.read_data()
        sys.stdout.write(
            "    %4d     %4d     %4d     %4d     %4d     %4d\r"%tuple(
            rc_data[:6]))
        sys.stdout.flush()
except(KeyboardInterrupt, SystemExit):
    rrx.close_serial()
except(Exception) as ex:
    print ex
    rrx.close_serial()
