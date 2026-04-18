import RPi.GPIO as GPIO
import time
import sleep
GPIO.setwarnings(False)
GPIO.cleanup()
# GPIO.setmode(GPIO.BCM)
# GPIO.setup(13, GPIO.OUT)
# GPIO.setup(12, GPIO.OUT)
# try:
#     while True:
#         GPIO.output(13, GPIO.HIGH)
#         GPIO.output(12, GPIO.HIGH)
#         time.sleep(2)
#         print("High")
#         GPIO.output(13, GPIO.LOW)
#         GPIO.output(12, GPIO.LOW)
#         time.sleep(2)
#         print("Low")
# except KeyboardInterrupt:
#     GPIO.cleanup()

Motor1A = 2
Motor1B = 3
Motor1E = 13
 
def setup():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)              # GPIO Numbering
    GPIO.setup(Motor1A,GPIO.OUT)  # All pins as Outputs
    GPIO.setup(Motor1B,GPIO.OUT)
    GPIO.setup(Motor1E,GPIO.OUT)
 
def loop():
    # Going forwards
    GPIO.output(Motor1A,GPIO.HIGH)
    GPIO.output(Motor1B,GPIO.LOW)
    GPIO.output(Motor1E,GPIO.HIGH)
    print("Going forwards")
 
    sleep(5)
    # Going backwards
    GPIO.output(Motor1A,GPIO.LOW)
    GPIO.output(Motor1B,GPIO.HIGH)
    GPIO.output(Motor1E,GPIO.HIGH)
    print("Going backwards")
 
    sleep(5)
    # Stop
    GPIO.output(Motor1E,GPIO.LOW)
    GPIO.output(Motor1B,GPIO.LOW)
    print("Stop")

def destroy():
    GPIO.cleanup()

if name == 'main':     # Program start from here
    setup()
    try:
            loop()
    except KeyboardInterrupt:
        destroy()