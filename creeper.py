"""
creeper.py — Main runtime for the Creeper robot
Run: python3 creeper.py

States:
  IDLE    — wanders randomly, scanning for a person
  CHASING — moves toward detected person, smooth turning
  PRIMED  — person within 50cm, creeper hiss + blink white for 1.5s
  EXPLODE — flashes white rapidly, then freezes
  FROZEN  — waits for ultrasonic reset (distance ~0cm)

LED: 4x white LEDs on GPIO14
Buzzer: passive buzzer on GPIO20
Restart: hold hand ~0cm from either ultrasonic sensor
"""

import time
import sys
import os
import threading
import random

sys.path.insert(0, os.path.dirname(__file__))
import config

import onnxruntime as ort
import numpy as np
import cv2
import RPi.GPIO as GPIO

# ── State constants ────────────────────────────────────────────
IDLE    = "IDLE"
CHASING = "CHASING"
PRIMED  = "PRIMED"
EXPLODE = "EXPLODE"
FROZEN  = "FROZEN"

# ── Shared state ───────────────────────────────────────────────
state           = IDLE
person_detected = False
person_offset_x = 0.0
dist_front      = None
dist_back       = None

# ── GPIO init ──────────────────────────────────────────────────
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# ── LED controller ─────────────────────────────────────────────
class LEDController:
    def __init__(self):
        GPIO.setup(config.LED_WHITE, GPIO.OUT)
        GPIO.output(config.LED_WHITE, GPIO.LOW)
        self.pwm = GPIO.PWM(config.LED_WHITE, 200)
        self.pwm.start(0)

    def off(self):
        self.pwm.ChangeDutyCycle(0)

    def on(self, brightness=100):
        self.pwm.ChangeDutyCycle(brightness)

    def flash(self, duration=0.3, brightness=100):
        self.on(brightness)
        time.sleep(duration)
        self.off()

    def blink_white(self, duration=1.5, interval=0.2):
        deadline = time.time() + duration
        while time.time() < deadline:
            self.on(100)
            time.sleep(interval / 2)
            self.off()
            time.sleep(interval / 2)

    def explode(self, flashes=16, interval=0.08):
        for i in range(flashes):
            self.on(100) if i % 2 == 0 else self.off()
            time.sleep(interval)
        self.off()

    def cleanup(self):
        self.off()
        self.pwm.stop()


# ── Buzzer controller ──────────────────────────────────────────
class BuzzerController:
    def __init__(self):
        GPIO.setup(config.BUZZER_PIN, GPIO.OUT)
        GPIO.output(config.BUZZER_PIN, GPIO.LOW)
        self.pwm = GPIO.PWM(config.BUZZER_PIN, 100)
        self.pwm.start(0)

    def tone(self, freq, duty=40):
        self.pwm.ChangeFrequency(max(50, freq))
        self.pwm.ChangeDutyCycle(duty)

    def off(self):
        self.pwm.ChangeDutyCycle(0)

    def play_hiss_burst(self, start_freq, end_freq, duration, steps, led, brightness):
        step_time = duration / steps
        for i in range(steps):
            freq = int(start_freq + (end_freq - start_freq) * (i / steps))
            self.tone(freq)
            led.on(brightness)
            time.sleep(step_time)
        self.off()
        led.off()

    def play_creeper_hiss(self, dist, led):
        min_dist = 5.0
        max_dist = 50.0
        dist     = max(min_dist, min(dist, max_dist))
        ratio    = (max_dist - dist) / (max_dist - min_dist)

        num_bursts = max(1, int(1 + ratio * 3))
        speed      = 1.0 - ratio * 0.6
        brightness = int(25 + ratio * 75)
        gap        = 0.12 - ratio * 0.08

        stages = [
            (800, 200, 0.18, 30),
            (700, 150, 0.15, 25),
            (600, 100, 0.12, 20),
            (500,  80, 0.10, 18),
        ]

        for i in range(num_bursts):
            start_freq, end_freq, duration, steps = stages[min(i, len(stages) - 1)]
            self.play_hiss_burst(
                start_freq, end_freq,
                duration * speed, steps,
                led, brightness
            )
            if i < num_bursts - 1:
                time.sleep(gap)

    def play_explosion(self, led):
        explosion_seq = [
            (800, 0.06), (400, 0.06), (900, 0.05), (300, 0.05),
            (1000, 0.04), (200, 0.04), (1100, 0.03), (150, 0.03),
            (1200, 0.03), (100, 0.03), (1300, 0.02), (80,  0.02),
            (1400, 0.02), (60,  0.02), (1500, 0.02), (50,  0.02),
        ]
        for i, (freq, duration) in enumerate(explosion_seq):
            self.tone(freq)
            led.on(100) if i % 2 == 0 else led.off()
            time.sleep(duration)
        self.off()
        led.off()

    def cleanup(self):
        self.off()
        self.pwm.stop()


# ── Motor controller ───────────────────────────────────────────
class MotorController:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        all_pins = [
            config.MOTOR_F_1, config.MOTOR_F_2,
            config.MOTOR_F_3, config.MOTOR_F_4,
            config.MOTOR_B_1, config.MOTOR_B_2,
            config.MOTOR_B_3, config.MOTOR_B_4,
            config.MOTOR_F_EnA, config.MOTOR_F_EnB,
            config.MOTOR_B_EnA, config.MOTOR_B_EnB,
        ]
        for pin in all_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)

        self.pwm_fl = GPIO.PWM(config.MOTOR_F_EnA, config.MOTOR_PWM_FREQ)
        self.pwm_fr = GPIO.PWM(config.MOTOR_F_EnB, config.MOTOR_PWM_FREQ)
        self.pwm_bl = GPIO.PWM(config.MOTOR_B_EnA, config.MOTOR_PWM_FREQ)
        self.pwm_br = GPIO.PWM(config.MOTOR_B_EnB, config.MOTOR_PWM_FREQ)
        for pwm in [self.pwm_fl, self.pwm_fr, self.pwm_bl, self.pwm_br]:
            pwm.start(0)

    def _set_left(self, fwd, speed):
        GPIO.output(config.MOTOR_F_1, GPIO.HIGH if fwd else GPIO.LOW)
        GPIO.output(config.MOTOR_F_2, GPIO.LOW  if fwd else GPIO.HIGH)
        GPIO.output(config.MOTOR_B_3, GPIO.HIGH if fwd else GPIO.LOW)
        GPIO.output(config.MOTOR_B_4, GPIO.LOW  if fwd else GPIO.HIGH)
        self.pwm_fl.ChangeDutyCycle(speed)
        self.pwm_bl.ChangeDutyCycle(speed)

    def _set_right(self, fwd, speed):
        GPIO.output(config.MOTOR_F_3, GPIO.HIGH if fwd else GPIO.LOW)
        GPIO.output(config.MOTOR_F_4, GPIO.LOW  if fwd else GPIO.HIGH)
        GPIO.output(config.MOTOR_B_1, GPIO.HIGH if fwd else GPIO.LOW)
        GPIO.output(config.MOTOR_B_2, GPIO.LOW  if fwd else GPIO.HIGH)
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
    GPIO.setup(config.TRIG_PIN,       GPIO.OUT)
    GPIO.setup(config.ECHO_PIN_FRONT, GPIO.IN)
    GPIO.setup(config.ECHO_PIN_BACK,  GPIO.IN)

    while True:
        dist_front = _measure_distance(config.ECHO_PIN_FRONT)
        time.sleep(0.01)
        dist_back  = _measure_distance(config.ECHO_PIN_BACK)
        time.sleep(0.05)


# ── Vision ─────────────────────────────────────────────────────
def load_model():
    model_path = os.path.join(os.path.dirname(__file__), "yolov8n.onnx")
    if not os.path.exists(model_path):
        print("[Vision] FAIL — yolov8n.onnx not found. Exiting.")
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


def vision_thread():
    global person_detected, person_offset_x
    print("[Vision] Loading YOLO model...")
    session, input_name = load_model()

    print("[Vision] Opening USB camera...")
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  config.CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS,          config.CAMERA_FPS)

    if not cap.isOpened():
        print("[Vision] FAIL — could not open camera")
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
            person_detected = True
            person_offset_x = ((x1 + x2) / 2) - frame_cx
        else:
            person_detected = False
            person_offset_x = 0.0


# ── Idle wandering ─────────────────────────────────────────────
def idle_wander(mc):
    action   = random.choice(["forward", "turn_left", "turn_right", "stop"])
    duration = random.uniform(0.5, 2.0)
    if action == "forward":
        mc.forward(40)
    elif action == "turn_left":
        mc.turn_left(40)
    elif action == "turn_right":
        mc.turn_right(40)
    else:
        mc.stop()
    time.sleep(duration)


# ── Restart check ──────────────────────────────────────────────
RESTART_DISTANCE_CM = 3

def check_restart():
    if dist_front is not None and dist_front < RESTART_DISTANCE_CM:
        return True
    if dist_back is not None and dist_back < RESTART_DISTANCE_CM:
        return True
    return False


# ── Main loop ──────────────────────────────────────────────────
def main():
    global state

    mc     = MotorController()
    led    = LEDController()
    buzzer = BuzzerController()

    print("[Creeper] Starting threads...")
    threading.Thread(target=ultrasonic_thread, daemon=True).start()
    threading.Thread(target=vision_thread,     daemon=True).start()

    print("[Creeper] Warming up...")
    time.sleep(2)
    print("[Creeper] Running. State: IDLE")

    try:
        while True:

            # ── IDLE ──────────────────────────────────────────
            if state == IDLE:
                led.off()
                buzzer.off()
                idle_wander(mc)

                if person_detected:
                    print("[Creeper] Person detected — CHASING")
                    led.flash(duration=0.3, brightness=100)
                    state = CHASING

            # ── CHASING ───────────────────────────────────────
            elif state == CHASING:
                if not person_detected:
                    print("[Creeper] Lost person — IDLE")
                    mc.stop()
                    state = IDLE
                    continue

                if dist_front is not None and dist_front <= 50:
                    print("[Creeper] Person within 50cm — PRIMED")
                    mc.stop()
                    state = PRIMED
                    continue

                mc.steer(person_offset_x, base_speed=55)
                time.sleep(0.05)

            # ── PRIMED ────────────────────────────────────────
            elif state == PRIMED:
                mc.stop()
                print("[Creeper] Hissing...")
                dist = dist_front if dist_front is not None else 50
                buzzer.play_creeper_hiss(dist, led)
                time.sleep(0.2)
                led.blink_white(duration=1.5, interval=0.2)
                print("[Creeper] EXPLODING")
                state = EXPLODE

            # ── EXPLODE ───────────────────────────────────────
            elif state == EXPLODE:
                mc.stop()
                buzzer.play_explosion(led)
                print("[Creeper] Frozen. Touch a sensor to restart.")
                state = FROZEN

            # ── FROZEN ────────────────────────────────────────
            elif state == FROZEN:
                mc.stop()
                led.off()
                buzzer.off()

                if check_restart():
                    print("[Creeper] Restart triggered — IDLE")
                    time.sleep(1)
                    state = IDLE
                else:
                    time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n[Creeper] Shutting down.")
    finally:
        mc.cleanup()
        led.cleanup()
        buzzer.cleanup()
        GPIO.cleanup()


if __name__ == "__main__":
    main()