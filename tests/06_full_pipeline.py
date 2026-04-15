"""
Stage 6 — Full pipeline integration
Run: python3 tests/06_full_pipeline.py
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
from tests.test04_motor_control import MotorController
from tests.test05_ultrasonic_test import read_distance_cm

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
    from ultralytics import YOLO
    model = YOLO("yolov8n.pt")

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

        results = model(bgr, classes=[0], verbose=False, imgsz=320)
        boxes = results[0].boxes

        if len(boxes) > 0 and float(boxes[0].conf[0]) >= config.CONFIDENCE_FLOOR:
            x1, y1, x2, y2 = boxes[0].xyxy[0].tolist()
            cx = (x1 + x2) / 2
            person_detected = True
            person_offset_x = cx - frame_cx
        else:
            person_detected = False
            person_offset_x = 0

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