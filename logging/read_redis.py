import redis
import sys

r = redis.StrictRedis('localhost')

print('  roll  droll | pitch dpitch |   yaw   dyaw |    ax     ay     az')
try:
    while(True):
        roll  = float(r.get('roll'))
        pitch = float(r.get('pitch'))
        yaw   = float(r.get('yaw'))
        droll = float(r.get('droll'))
        dpitch = float(r.get('dpitch'))
        dyaw = float(r.get('dyaw'))
        ax = float(r.get('ax'))
        ay = float(r.get('ay'))
        az = float(r.get('az'))
        sys.stdout.write(
            "%6.1f  %5.0f | "%(roll, droll) +
            "%5.1f  %5.0f | "%(pitch, dpitch) +
            "%5.1f  %5.0f | "%(yaw, dyaw) +
            "%5.0f  %5.0f  %5.0f\r"%(ax, ay, az))
        sys.stdout.flush()
except (KeyboardInterrupt, SystemExit):
    # Graceful Exit
    pass
