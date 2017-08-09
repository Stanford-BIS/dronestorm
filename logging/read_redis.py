import redis
import sys

r = redis.StrictRedis('192.168.0.165')

try:
    while(True):
        roll  = float(r.get('roll'))
        pitch = float(r.get('pitch'))
        yaw   = float(r.get('yaw'))
        sys.stdout.write(
            "roll:%6.1f pitch:%5.1f yaw:%5.1f\r"%(roll, pitch, yaw))
        sys.stdout.flush()
except (KeyboardInterrupt, SystemExit):
    # Graceful Exit
    pass
