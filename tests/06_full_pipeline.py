"""
Stage 6 — Full pipeline integration
Run: python3 tests/06_full_pipeline.py
Behaviour:
  - Detects person via YOLOv8n ONNX -> drives toward them
  - Ultrasonic < OBSTACLE_DISTANCE_CM -> stops regardless of detection
  - Ctrl+C to quit cleanly
"""

import time, sys, os, threading
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
import config

from picamera2 import Picamera2
import onnxruntime as ort
import numpy as np
import cv2
import RPi.GPIO as GPIO

# ── Shared state ───────────────────────────────────────────────
obstacle_detected = False
person_detected   = False
person_offset_x   = 0  # pixels from frame centre; negative=left, positive=right

# ── Motor controller ───────────────────────────────────────────
class MotorController:
    def __init__(self):
        self.pi = pigpio.pi()

    def forward(self, speed=60):
        self.pi.hardware_PWM(config.MOTOR_FL_EnB, config.MOTOR_PWM_FREQ, int(speed * 10000))
        self.pi.hardware_PWM(config.MOTOR_FR_EnA, config.MOTOR_PWM_FREQ, int(speed * 10000))

    def stop(self):
        self.pi.hardware_PWM(config.MOTOR_FL_EnB, config.MOTOR_PWM_FREQ, 0)
        self.pi.hardware_PWM(config.MOTOR_FR_EnA, config.MOTOR_PWM_FREQ, 0)

    def turn_left(self, speed=50):
        self.pi.hardware_PWM(config.MOTOR_FL_EnB, config.MOTOR_PWM_FREQ, 0)
        self.pi.hardware_PWM(config.MOTOR_FR_EnA, config.MOTOR_PWM_FREQ, int(speed * 10000))

    def turn_right(self, speed=50):
        self.pi.hardware_PWM(config.MOTOR_FL_EnB, config.MOTOR_PWM_FREQ, int(speed * 10000))
        self.pi.hardware_PWM(config.MOTOR_FR_EnA, config.MOTOR_PWM_FREQ, 0)

    def cleanup(self):
        self.stop()
        self.pi.stop()

# ── ONNX model ─────────────────────────────────────────────────
def load_model():
    model_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "yolov8n.onnx")
    if not os.path.exists(model_path):
        print(f"[Stage 6] FAIL — yolov8n.onnx not found at {model_path}")
        sys.exit(1)
    session = ort.InferenceSession(model_path)
    input_name = session.get_inputs()[0].name
    return session, input_name

def detect_people(session, input_name, bgr):
    img = cv2.resize(bgr, (320, 320))
    img_input = img.transpose(2, 0, 1)[np.newaxis].astype(np.float32) / 255.0
    outputs = session.run(None, {input_name: img_input})[0][0]

    boxes = []
    for i in range(outputs.shape[1]):
        scores = outputs[4:, i]
        class_id = int(np.argmax(scores))
        conf = float(scores[class_id])
        if class_id != 0 or conf < config.CONFIDENCE_FLOOR:
            continue
        cx, cy, w, h = outputs[:4, i]
        x1 = int((cx - w/2) * (config.CAMERA_WIDTH / 320))
        y1 = int((cy - h/2) * (config.CAMERA_HEIGHT / 320))
        x2 = int((cx + w/2) * (config.CAMERA_WIDTH / 320))
        y2 = int((cy + h/2) * (config.CAMERA_HEIGHT / 320))
        boxes.append((x1, y1, x2, y2, conf))
    return boxes

# ── Threads ────────────────────────────────────────────────────
def _measure_distance(echo_pin):
    """Fire the shared trigger and measure echo on the given pin."""
    GPIO.output(config.TRIG_PIN, False)
    time.sleep(0.002)
    GPIO.output(config.TRIG_PIN, True)
    time.sleep(0.00001)
    GPIO.output(config.TRIG_PIN, False)

    timeout = time.time() + 0.04
    pulse_start = time.time()
    while GPIO.input(echo_pin) == 0:
        pulse_start = time.time()
        if time.time() > timeout:
            return None          # sensor didn't respond

    timeout = time.time() + 0.04
    pulse_end = pulse_start
    while GPIO.input(echo_pin) == 1:
        pulse_end = time.time()
        if time.time() > timeout:
            return None          # echo never went low

    return (pulse_end - pulse_start) * 17150

def ultrasonic_thread():
    global obstacle_detected_front, obstacle_detected_back

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(config.TRIG_PIN,       GPIO.OUT)
    GPIO.setup(config.ECHO_PIN_FRONT, GPIO.IN)
    GPIO.setup(config.ECHO_PIN_BACK,  GPIO.IN)

    while True:
        # --- front sensor ---
        dist_front = _measure_distance(config.ECHO_PIN_FRONT)
        if dist_front is not None:
            obstacle_detected_front = dist_front < config.OBSTACLE_DISTANCE_CM

        time.sleep(0.01)   # small gap between triggers avoids cross-talk

        # --- back sensor ---
        dist_back = _measure_distance(config.ECHO_PIN_BACK)
        if dist_back is not None:
            obstacle_detected_back = dist_back < config.OBSTACLE_DISTANCE_CM

        time.sleep(0.05)

def detection_thread():
    global person_detected, person_offset_x
    print("[Stage 6] Loading ONNX model...")
    session, input_name = load_model()

    print("[Stage 6] Starting camera...")
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
        boxes = detect_people(session, input_name, bgr)

        if boxes:
            x1, y1, x2, y2, conf = boxes[0]
            cx = (x1 + x2) / 2
            person_detected = True
            person_offset_x = cx - frame_cx
        else:
            person_detected = False
            person_offset_x = 0

# ── Control loop ───────────────────────────────────────────────
def control_loop(mc):
    DEAD_ZONE = 60
    print("[Stage 6] Control loop running. Ctrl+C to stop.")
    while True:
        if obstacle_detected:
            mc.stop()
            print("[Stage 6] Obstacle — stopped")
        elif person_detected:
            if abs(person_offset_x) < DEAD_ZONE:
                mc.forward(50)
                print(f"[Stage 6] Forward  (offset={person_offset_x:.0f}px)")
            elif person_offset_x < 0:
                mc.turn_left(45)
                print(f"[Stage 6] Turn left  (offset={person_offset_x:.0f}px)")
            else:
                mc.turn_right(45)
                print(f"[Stage 6] Turn right (offset={person_offset_x:.0f}px)")
        else:
            mc.stop()
        time.sleep(0.1)

# ── Main ───────────────────────────────────────────────────────
def main():
    mc = MotorController()
    try:
        t_ultra  = threading.Thread(target=ultrasonic_thread, daemon=True)
        # t_detect = threading.Thread(target=detection_thread,  daemon=True)
        # t_ultra.start()
        # t_detect.start()
        # control_loop(mc)
    except KeyboardInterrupt:
        print("\n[Stage 6] Shutting down.")
    finally:
        mc.cleanup()

if __name__ == "__main__":
    main()