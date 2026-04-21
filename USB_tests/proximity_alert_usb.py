"""
Proximity + Person Alert — Creeper Edition
Triggers white LED and creeper hiss when:
  - YOLO detects a person via camera AND
  - Ultrasonic sensor reads within ALERT_DISTANCE_CM
Hiss intensity and LED brightness increase as person gets closer.
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

GPIO.setup(config.TRIG_PIN,       GPIO.OUT)
GPIO.setup(config.ECHO_PIN_FRONT, GPIO.IN)
GPIO.setup(config.LED_WHITE,      GPIO.OUT)
GPIO.setup(config.BUZZER_PIN,     GPIO.OUT)

GPIO.output(config.TRIG_PIN,   GPIO.LOW)
GPIO.output(config.LED_WHITE,  GPIO.LOW)
GPIO.output(config.BUZZER_PIN, GPIO.LOW)

pwm_led    = GPIO.PWM(config.LED_WHITE,  200)
pwm_buzzer = GPIO.PWM(config.BUZZER_PIN, 100)
pwm_led.start(0)
pwm_buzzer.start(0)

# ── Shared state ───────────────────────────────────────────────
distance_cm     = None
person_detected = False

# ── ONNX model ─────────────────────────────────────────────────
def load_model():
    model_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "yolov8n.onnx")
    if not os.path.exists(model_path):
        print("[Creeper] FAIL — yolov8n.onnx not found")
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
    print("[Creeper] Loading YOLO model...")
    session, input_name = load_model()

    print("[Creeper] Opening camera...")
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  config.CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS,          config.CAMERA_FPS)

    if not cap.isOpened():
        print("[Creeper] FAIL — could not open camera")
        sys.exit(1)

    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        person_detected = detect_people(session, input_name, frame)

# ── Creeper hiss ───────────────────────────────────────────────
# The Minecraft creeper hiss is a descending noise burst —
# modelled as rapid frequency sweeps from high to low with
# short silences between bursts, getting faster and louder
# as the person gets closer.

HISS_STAGES = [
    # (start_freq, end_freq, duration, steps) — one burst
    (800,  200, 0.18, 30),
    (700,  150, 0.15, 25),
    (600,  100, 0.12, 20),
    (500,   80, 0.10, 18),
]

def play_hiss_burst(start_freq, end_freq, duration, steps, brightness):
    """Play a single descending frequency sweep — one hiss burst."""
    step_time = duration / steps
    for i in range(steps):
        freq = int(start_freq + (end_freq - start_freq) * (i / steps))
        freq = max(50, freq)  # buzzer minimum ~50Hz
        pwm_buzzer.ChangeFrequency(freq)
        pwm_buzzer.ChangeDutyCycle(40)
        pwm_led.ChangeDutyCycle(brightness)
        time.sleep(step_time)
    pwm_buzzer.ChangeDutyCycle(0)
    pwm_led.ChangeDutyCycle(0)

def play_creeper_hiss(dist):
    """
    Full creeper hiss sequence.
    Closer distance = more bursts played + faster + brighter.
    """
    min_dist = 5.0
    max_dist = float(config.ALERT_DISTANCE_CM)
    dist     = max(min_dist, min(dist, max_dist))
    ratio    = (max_dist - dist) / (max_dist - min_dist)  # 0.0 far, 1.0 close

    # Number of hiss bursts — 1 at far range, up to 4 when very close
    num_bursts  = max(1, int(1 + ratio * 3))

    # Speed multiplier — closer = faster bursts
    speed       = 1.0 - ratio * 0.6   # 1.0 far, 0.4 close

    # Brightness — 25% far, 100% close
    brightness  = int(25 + ratio * 75)

    # Gap between bursts — shorter when closer
    gap         = 0.12 - ratio * 0.08  # 0.12s far, 0.04s close

    for i in range(num_bursts):
        stage = HISS_STAGES[min(i, len(HISS_STAGES) - 1)]
        start_freq, end_freq, duration, steps = stage
        play_hiss_burst(
            start_freq,
            end_freq,
            duration * speed,
            steps,
            brightness
        )
        if i < num_bursts - 1:
            time.sleep(gap)

def alert_off():
    pwm_led.ChangeDutyCycle(0)
    pwm_buzzer.ChangeDutyCycle(0)

# ── Main loop ──────────────────────────────────────────────────
def main():
    print(f"[Creeper] Starting. Trigger distance: {config.ALERT_DISTANCE_CM}cm")
    print("[Creeper] Waiting for camera and sensor to warm up...")

    threading.Thread(target=ultrasonic_thread, daemon=True).start()
    threading.Thread(target=vision_thread,     daemon=True).start()
    time.sleep(3)

    print("[Creeper] Running. Ctrl+C to stop.")

    try:
        while True:
            dist = distance_cm

            if dist is None:
                print("[Creeper] Sensor timeout", end="\r")
                alert_off()
                time.sleep(0.1)
                continue

            print(f"[Creeper] Distance: {dist:.1f}cm | Person: {person_detected}    ", end="\r")

            if person_detected and dist <= config.ALERT_DISTANCE_CM:
                play_creeper_hiss(dist)
                # Brief pause between hiss cycles
                time.sleep(0.3)
            else:
                alert_off()
                time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n[Creeper] Shutting down.")
    finally:
        alert_off()
        pwm_led.stop()
        pwm_buzzer.stop()
        GPIO.cleanup()

if __name__ == "__main__":
    main()