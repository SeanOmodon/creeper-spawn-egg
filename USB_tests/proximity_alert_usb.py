"""
Proximity + Person Alert
Triggers white LED and dramatic buzzer when:
  - YOLO detects a person via camera AND
  - Ultrasonic sensor reads within ALERT_DISTANCE_CM
LED brightness and buzzer speed increase as person gets closer.
Run: python3 USB_tests/proximity_alert_usb.py
"""

import time, sys, os, threading
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
import config

import RPi.GPIO as GPIO
import onnxruntime as ort
import numpy as np
import cv2

# ── GPIO setup ─────────────────────────────────────────────────
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

GPIO.setup(config.TRIG_PIN,        GPIO.OUT)
GPIO.setup(config.ECHO_PIN_FRONT,  GPIO.IN)
GPIO.setup(config.LED_WHITE,       GPIO.OUT)
GPIO.setup(config.BUZZER_PIN,      GPIO.OUT)

GPIO.output(config.TRIG_PIN,   GPIO.LOW)
GPIO.output(config.LED_WHITE,  GPIO.LOW)
GPIO.output(config.BUZZER_PIN, GPIO.LOW)

pwm_led    = GPIO.PWM(config.LED_WHITE,  200)
pwm_buzzer = GPIO.PWM(config.BUZZER_PIN, 1000)
pwm_led.start(0)
pwm_buzzer.start(0)

# ── Shared state ───────────────────────────────────────────────
distance_cm     = None
person_detected = False

# ── ONNX model ─────────────────────────────────────────────────
def load_model():
    model_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "yolov8n.onnx")
    if not os.path.exists(model_path):
        print("[Alert] FAIL — yolov8n.onnx not found")
        sys.exit(1)
    session    = ort.InferenceSession(model_path)
    input_name = session.get_inputs()[0].name
    return session, input_name

def detect_people(session, input_name, bgr):
    img       = cv2.resize(bgr, (320, 320))
    img_input = img.transpose(2, 0, 1)[np.newaxis].astype(np.float32) / 255.0
    outputs   = session.run(None, {input_name: img_input})[0][0]
    for i in range(outputs.shape[1]):
        scores   = outputs[4:, i]
        class_id = int(np.argmax(scores))
        conf     = float(scores[class_id])
        if class_id == 0 and conf >= config.CONFIDENCE_FLOOR:
            return True
    return False

# ── Ultrasonic thread ──────────────────────────────────────────
def ultrasonic_thread():
    global distance_cm
    while True:
        GPIO.output(config.TRIG_PIN, False)
        time.sleep(0.002)
        GPIO.output(config.TRIG_PIN, True)
        time.sleep(0.00001)
        GPIO.output(config.TRIG_PIN, False)

        timeout     = time.time() + 0.04
        pulse_start = time.time()
        while GPIO.input(config.ECHO_PIN_FRONT) == 0:
            pulse_start = time.time()
            if time.time() > timeout:
                distance_cm = None
                break
        else:
            timeout   = time.time() + 0.04
            pulse_end = pulse_start
            while GPIO.input(config.ECHO_PIN_FRONT) == 1:
                pulse_end = time.time()
                if time.time() > timeout:
                    break
            distance_cm = (pulse_end - pulse_start) * 17150

        time.sleep(0.06)

# ── Vision thread ──────────────────────────────────────────────
def vision_thread():
    global person_detected
    print("[Alert] Loading YOLO model...")
    session, input_name = load_model()

    print("[Alert] Opening camera...")
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  config.CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS,          config.CAMERA_FPS)

    if not cap.isOpened():
        print("[Alert] FAIL — could not open camera")
        sys.exit(1)

    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        person_detected = detect_people(session, input_name, frame)

# ── Alert helpers ──────────────────────────────────────────────
def get_interval(dist):
    """Faster blink as person gets closer."""
    min_dist = 5.0
    max_dist = float(config.ALERT_DISTANCE_CM)
    dist     = max(min_dist, min(dist, max_dist))
    ratio    = (max_dist - dist) / (max_dist - min_dist)
    return config.ALERT_MAX_INTERVAL - ratio * (
        config.ALERT_MAX_INTERVAL - config.ALERT_MIN_INTERVAL
    )

def get_brightness(dist):
    """Brighter LED as person gets closer — 30% to 100%."""
    min_dist = 5.0
    max_dist = float(config.ALERT_DISTANCE_CM)
    dist     = max(min_dist, min(dist, max_dist))
    ratio    = (max_dist - dist) / (max_dist - min_dist)
    return 30 + ratio * 70

def get_buzzer_freq(dist):
    """
    Dramatic rising pitch as person gets closer.
    500Hz at trigger distance -> 2500Hz at 5cm
    """
    min_dist = 5.0
    max_dist = float(config.ALERT_DISTANCE_CM)
    dist     = max(min_dist, min(dist, max_dist))
    ratio    = (max_dist - dist) / (max_dist - min_dist)
    return int(500 + ratio * 2000)

def alert_off():
    pwm_led.ChangeDutyCycle(0)
    pwm_buzzer.ChangeDutyCycle(0)

# ── Main loop ──────────────────────────────────────────────────
def main():
    print(f"[Alert] Starting. Trigger distance: {config.ALERT_DISTANCE_CM}cm")
    print("[Alert] Waiting for camera and sensor to warm up...")

    threading.Thread(target=ultrasonic_thread, daemon=True).start()
    threading.Thread(target=vision_thread,     daemon=True).start()
    time.sleep(3)

    print("[Alert] Running. Ctrl+C to stop.")

    try:
        while True:
            dist = distance_cm

            if dist is None:
                print("[Alert] Sensor timeout", end="\r")
                alert_off()
                time.sleep(0.1)
                continue

            print(f"[Alert] Distance: {dist:.1f}cm | Person: {person_detected}    ", end="\r")

            if person_detected and dist <= config.ALERT_DISTANCE_CM:
                interval   = get_interval(dist)
                brightness = get_brightness(dist)
                freq       = get_buzzer_freq(dist)
                half       = interval / 2

                # Update buzzer frequency dynamically
                pwm_buzzer.ChangeFrequency(freq)

                # Flash ON
                pwm_led.ChangeDutyCycle(brightness)
                pwm_buzzer.ChangeDutyCycle(50)
                time.sleep(half)

                # Flash OFF
                alert_off()
                time.sleep(half)

            else:
                alert_off()
                time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n[Alert] Shutting down.")
    finally:
        alert_off()
        pwm_led.stop()
        pwm_buzzer.stop()
        GPIO.cleanup()

if __name__ == "__main__":
    main()