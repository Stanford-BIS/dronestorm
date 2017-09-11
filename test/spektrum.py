# Tests the spektrum communication module
import sys
from dronestorm.comm.spektrum import SpektrumRemoteReceiver

print("Throttle     Roll    Pitch      Yaw     AUX1     AUX2")

rrx = SpektrumRemoteReceiver()

try:
    rrx.align_serial()
    while True:
        rc_data = rrx.get_data()
        sys.stdout.write(
            "    %4d     %4d     %4d     %4d     %4d     %4d\r"%tuple(
            rc_data[:6]))
        sys.stdout.flush()
        # print(rc_data)

        # ser.write(data_buf)
except(KeyboardInterrupt, SystemExit):
    rrx.close_serial()
except(Exception) as ex:
    print ex
    rrx.close_serial()
