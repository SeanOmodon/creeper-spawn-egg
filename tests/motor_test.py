import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
GPIO.setup(13, GPIO.OUT)
GPIO.setup(12, GPIO.OUT)
try:
    while True:
        GPIO.output(13, GPIO.HIGH)
        GPIO.output(12, GPIO.HIGH)
        time.sleep(2)
        GPIO.output(13, GPIO.LOW)
        GPIO.output(12, GPIO.LOW)
        time.sleep(2)
except KeyboardInterrupt:
    GPIO.cleanup()