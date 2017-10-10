import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

TRIG_ID = 12
ECHO_ID = 16

GPIO.setup(TRIG_ID, GPIO.OUT)
GPIO.setup(ECHO_ID, GPIO.IN)

GPIO.output(TRIG_ID, False)
time.sleep(2)

try:
    while (True):
        GPIO.output(TRIG_ID, True)
        time.sleep(0.001)
        GPIO.output(TRIG_ID, False)
    
        edge_detect = GPIO.wait_for_edge(ECHO_ID, GPIO.RISING, timeout=100)
        if edge_detect is not None:
            pulse_start = time.time()
        else:
            continue
        edge_detect = GPIO.wait_for_edge(ECHO_ID, GPIO.FALLING, timeout=100)
        if edge_detect is not None:
            pulse_end = time.time()
        else:
            continue

        pulse_duration = pulse_end - pulse_start
    
        distance = pulse_duration * 17150
    
        distance = round(distance, 2)
    
        print "Distance:",distance,"cm"
    
        time.sleep(0.05)
except (KeyboardInterrupt, SystemExit):
    # Graceful Exit
    print("")
    GPIO.cleanup()
