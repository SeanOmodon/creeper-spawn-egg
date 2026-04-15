"""
Stage 5 — Full pipeline integration
Run: python3 tests/05_full_pipeline.py
Behaviour:
  - Detects person -> drives toward them
  - Ultrasonic < OBSTACLE_DISTANCE_CM -> stops regardless of detection
  - Ctrl+C to quit cleanly
"""

import time, sys, os, threading
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
import config

from picamera2 import Picamera2
import cv2
import RPi.GPIO as GPIO
from tests.test03_motor_control import MotorController
from tests.test04_ultrasonic_test import read_distance_cm

# ── Shared state ───────────────────────────────────────────────
obstacle_detected = False
person_detected   = False
person_offset_x   = 0          # pixels from frame centre; negative=left, positive=right

def ultrasonic_thread():
    global obstacle_detected
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(config.TRIG_PIN, GPIO.OUT)
    GPIO.setup(config.ECHO_PIN, GPIO.IN)
    while True:
        dist = read_distance_cm()
        obstacle_detected = (dist is not None and dist < config.OBSTACLE_DISTANCE_CM)
        time.sleep(0.05)

def detection_thread():
    global person_detected, person_offset_x
    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    cam = Picamera2()
    cam.configure(cam.create_video_configuration(
        main={"size": (config.CAMERA_WIDTH, config.CAMERA_HEIGHT), "format": "RGB888"},
        controls={"FrameRate": config.CAMERA_FPS}
    ))
    cam.start()
    time.sleep(1)
    frame_cx = config.CAMERA_WIDTH / 2

    while True:
        frame = cam.capture_array()
        bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        small = cv2.resize(bgr, (320, 240))
        rects, weights = hog.detectMultiScale(
            small,
            winStride=config.HOG_WIN_STRIDE,
            padding=config.HOG_PADDING,
            scale=config.HOG_SCALE
        )
        if len(rects) > 0 and weights[0][0] >= config.CONFIDENCE_FLOOR:
            x, y, w, h = rects[0]
            cx = (x + w / 2) * (config.CAMERA_WIDTH / 320)
            person_detected  = True
            person_offset_x  = cx - frame_cx
        else:
            person_detected  = False
            person_offset_x  = 0

def control_loop(mc):
    DEAD_ZONE = 60  # px — ignore small offsets
    print("[Stage 5] Control loop running. Ctrl+C to stop.")
    while True:
        if obstacle_detected:
            mc.stop()
            print("[Stage 5] Obstacle — stopped")
        elif person_detected:
            if abs(person_offset_x) < DEAD_ZONE:
                mc.forward(50)
                print(f"[Stage 5] Forward  (offset={person_offset_x:.0f}px)")
            elif person_offset_x < 0:
                mc.turn_left(45)
                print(f"[Stage 5] Turn left (offset={person_offset_x:.0f}px)")
            else:
                mc.turn_right(45)
                print(f"[Stage 5] Turn right (offset={person_offset_x:.0f}px)")
        else:
            mc.stop()
        time.sleep(0.1)

def main():
    mc = MotorController()
    try:
        t_ultra = threading.Thread(target=ultrasonic_thread, daemon=True)
        t_detect = threading.Thread(target=detection_thread, daemon=True)
        t_ultra.start()
        t_detect.start()
        control_loop(mc)
    except KeyboardInterrupt:
        print("\n[Stage 5] Shutting down.")
    finally:
        mc.cleanup()

if __name__ == "__main__":
    main()