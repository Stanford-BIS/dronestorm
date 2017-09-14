# Tests the spektrum communication module
import sys
import time
from dronestorm.comm import SpektrumRemoteReceiver

N=100
print("Test writing over spektrum remote receiver protocol")
print("Please ensure that:")
print("  - the pc is connected properly to the flight control board")

rrx = SpektrumRemoteReceiver()
try:
    data = [1300 + i*100 for i in range(6)]
    while True:
        rrx.write_data(data)
        time.sleep(0.01)
except(KeyboardInterrupt, SystemExit):
    rrx.close_serial()
except(Exception) as ex:
    print ex
    rrx.close_serial()
