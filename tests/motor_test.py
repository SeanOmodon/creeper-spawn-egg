import RPi.GPIO as GPIO
import time
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

# EN B of Front
# Motor1A = 2
# Motor1B = 3
# Motor1E = 13

# EN A of Front
Motor1A = 0
Motor1B = 1
Motor1E = 12
 
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
 
    time.sleep(5)
    # Going backwards
    GPIO.output(Motor1A,GPIO.LOW)
    GPIO.output(Motor1B,GPIO.HIGH)
    GPIO.output(Motor1E,GPIO.HIGH)
    print("Going backwards")
 
    time.sleep(5)
    # Stop
    GPIO.output(Motor1E,GPIO.LOW)
    GPIO.output(Motor1B,GPIO.LOW)
    print("Stop")

def destroy():
    GPIO.cleanup()

    # Program start from here
setup()
try:
    while True:
        loop()
except KeyboardInterrupt:
    destroy()