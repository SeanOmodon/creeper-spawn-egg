"""
Stage 6 — Full pipeline integration (USB camera)
Run: python3 tests/06_full_pipeline.py
Behaviour:
  - Detects person via YOLOv8n ONNX -> drives toward them using smooth steering
  - Ultrasonic front sensor triggers stop/explode at 50cm
  - Ctrl+C to quit cleanly
"""

import time, sys, os, threading
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
import config

import onnxruntime as ort
import numpy as np
import cv2
import RPi.GPIO as GPIO

# ── Shared state ───────────────────────────────────────────────
person_detected  = False
person_offset_x  = 0.0      # px from frame centre, negative=left
dist_front       = None     # cm
dist_back        = None     # cm

# ── Motor controller ───────────────────────────────────────────
class MotorController:
    def __init__(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        all_pins = [
            config.MOTOR_FL_1, config.MOTOR_FL_2,
            config.MOTOR_FR_1, config.MOTOR_FR_2,
            config.MOTOR_BR_1, config.MOTOR_BR_2,
            config.MOTOR_BL_1, config.MOTOR_BL_2,
            config.MOTOR_FL_EnB, config.MOTOR_FR_EnA,
            config.MOTOR_BL_EnA, config.MOTOR_BR_EnB,
        ]
        for pin in all_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)

        self.pwm_fl = GPIO.PWM(config.MOTOR_FL_EnB, config.MOTOR_PWM_FREQ)
        self.pwm_fr = GPIO.PWM(config.MOTOR_FR_EnA, config.MOTOR_PWM_FREQ)
        self.pwm_bl = GPIO.PWM(config.MOTOR_BL_EnA, config.MOTOR_PWM_FREQ)
        self.pwm_br = GPIO.PWM(config.MOTOR_BR_EnB, config.MOTOR_PWM_FREQ)
        for pwm in [self.pwm_fl, self.pwm_fr, self.pwm_bl, self.pwm_br]:
            pwm.start(0)

    def _set_left(self, fwd, speed):
        GPIO.output(config.MOTOR_FL_1, GPIO.HIGH if fwd else GPIO.LOW)
        GPIO.output(config.MOTOR_FL_2, GPIO.LOW  if fwd else GPIO.HIGH)
        GPIO.output(config.MOTOR_BL_1, GPIO.HIGH if fwd else GPIO.LOW)
        GPIO.output(config.MOTOR_BL_2, GPIO.LOW  if fwd else GPIO.HIGH)
        self.pwm_fl.ChangeDutyCycle(speed)
        self.pwm_bl.ChangeDutyCycle(speed)

    def _set_right(self, fwd, speed):
        GPIO.output(config.MOTOR_FR_1, GPIO.HIGH if fwd else GPIO.LOW)
        GPIO.output(config.MOTOR_FR_2, GPIO.LOW  if fwd else GPIO.HIGH)
        GPIO.output(config.MOTOR_BR_1, GPIO.HIGH if fwd else GPIO.LOW)
        GPIO.output(config.MOTOR_BR_2, GPIO.LOW  if fwd else GPIO.HIGH)
        self.pwm_fr.ChangeDutyCycle(speed)
        self.pwm_br.ChangeDutyCycle(speed)

    def forward(self, speed=60):
        self._set_left(True,  speed)
        self._set_right(True, speed)

    def backward(self, speed=60):
        self._set_left(False,  speed)
        self._set_right(False, speed)

    def stop(self):
        for pwm in [self.pwm_fl, self.pwm_fr, self.pwm_bl, self.pwm_br]:
            pwm.ChangeDutyCycle(0)

    def turn_left(self, speed=50):
        self._set_left(False, speed)
        self._set_right(True, speed)

    def turn_right(self, speed=50):
        self._set_left(True,  speed)
        self._set_right(False, speed)

    def steer(self, offset_x, base_speed=55):
        dead_zone  = 60
        frame_half = config.CAMERA_WIDTH / 2

        if abs(offset_x) < dead_zone:
            self.forward(base_speed)
            return

        beyond     = abs(offset_x) - dead_zone
        max_range  = frame_half - dead_zone
        factor     = min(beyond / max_range, 1.0)
        slow_speed = base_speed * (1.0 - 0.8 * factor)

        if offset_x < 0:
            self._set_left(True,  slow_speed)
            self._set_right(True, base_speed)
        else:
            self._set_left(True,  base_speed)
            self._set_right(True, slow_speed)

    def cleanup(self):
        self.stop()
        for pwm in [self.pwm_fl, self.pwm_fr, self.pwm_bl, self.pwm_br]:
            pwm.stop()
        GPIO.cleanup()

# ── ONNX model ─────────────────────────────────────────────────
def load_model():
    model_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "yolov8n.onnx")
    if not os.path.exists(model_path):
        print(f"[Stage 6] FAIL — yolov8n.onnx not found at {model_path}")
        sys.exit(1)
    session    = ort.InferenceSession(model_path)
    input_name = session.get_inputs()[0].name
    return session, input_name

def detect_people(session, input_name, bgr):
    img       = cv2.resize(bgr, (320, 320))
    img_input = img.transpose(2, 0, 1)[np.newaxis].astype(np.float32) / 255.0
    outputs   = session.run(None, {input_name: img_input})[0][0]

    boxes = []
    for i in range(outputs.shape[1]):
        scores   = outputs[4:, i]
        class_id = int(np.argmax(scores))
        conf     = float(scores[class_id])
        if class_id != 0 or conf < config.CONFIDENCE_FLOOR:
            continue
        cx, cy, w, h = outputs[:4, i]
        x1 = int((cx - w / 2) * (config.CAMERA_WIDTH  / 320))
        y1 = int((cy - h / 2) * (config.CAMERA_HEIGHT / 320))
        x2 = int((cx + w / 2) * (config.CAMERA_WIDTH  / 320))
        y2 = int((cy + h / 2) * (config.CAMERA_HEIGHT / 320))
        boxes.append((x1, y1, x2, y2, conf))
    return boxes

# ── Ultrasonic ─────────────────────────────────────────────────
def _measure_distance(echo_pin, timeout=0.04):
    GPIO.output(config.TRIG_PIN, False)
    time.sleep(0.002)
    GPIO.output(config.TRIG_PIN, True)
    time.sleep(0.00001)
    GPIO.output(config.TRIG_PIN, False)

    deadline    = time.time() + timeout
    pulse_start = time.time()
    while GPIO.input(echo_pin) == 0:
        pulse_start = time.time()
        if time.time() > deadline:
            return None

    deadline  = time.time() + timeout
    pulse_end = pulse_start
    while GPIO.input(echo_pin) == 1:
        pulse_end = time.time()
        if time.time() > deadline:
            return None

    return (pulse_end - pulse_start) * 17150

def ultrasonic_thread():
    global dist_front, dist_back
    GPIO.setup(config.TRIG_PIN,        GPIO.OUT)
    GPIO.setup(config.ECHO_PIN_FRONT,  GPIO.IN)
    GPIO.setup(config.ECHO_PIN_BACK,   GPIO.IN)

    while True:
        dist_front = _measure_distance(config.ECHO_PIN_FRONT)
        time.sleep(0.01)
        dist_back  = _measure_distance(config.ECHO_PIN_BACK)
        time.sleep(0.05)

# ── Vision ─────────────────────────────────────────────────────
def vision_thread():
    global person_detected, person_offset_x
    print("[Stage 6] Loading ONNX model...")
    session, input_name = load_model()

    print("[Stage 6] Opening USB camera...")
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  config.CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS,          config.CAMERA_FPS)

    if not cap.isOpened():
        print("[Stage 6] FAIL — could not open camera")
        sys.exit(1)

    time.sleep(1)
    frame_cx = config.CAMERA_WIDTH / 2

    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        boxes = detect_people(session, input_name, frame)
        if boxes:
            x1, y1, x2, y2, conf = boxes[0]
            person_detected  = True
            person_offset_x  = ((x1 + x2) / 2) - frame_cx
        else:
            person_detected  = False
            person_offset_x  = 0.0

# ── Control loop ───────────────────────────────────────────────
def control_loop(mc):
    print("[Stage 6] Control loop running. Ctrl+C to stop.")
    while True:
        if person_detected:
            if dist_front is not None and dist_front <= config.OBSTACLE_DISTANCE_CM:
                mc.stop()
                print(f"[Stage 6] Person within {config.OBSTACLE_DISTANCE_CM}cm — stopped")
            else:
                mc.steer(person_offset_x, base_speed=55)
                print(f"[Stage 6] Steering (offset={person_offset_x:.0f}px)")
        else:
            mc.stop()
        time.sleep(0.1)

# ── Main ───────────────────────────────────────────────────────
def main():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)

    mc = MotorController()
    try:
        t_ultra  = threading.Thread(target=ultrasonic_thread, daemon=True)
        t_vision = threading.Thread(target=vision_thread,     daemon=True)
        t_ultra.start()
        t_vision.start()
        time.sleep(2)  # let threads warm up
        control_loop(mc)
    except KeyboardInterrupt:
        print("\n[Stage 6] Shutting down.")
    finally:
        mc.cleanup()

if __name__ == "__main__":
    main()
