"""
creeper.py — Main runtime for the Creeper robot
Run: python3 creeper.py

States:
  IDLE    — wanders randomly, scanning for a person
  CHASING — moves toward detected person, smooth steering
  PRIMED  — person within 50cm, creeper hiss + blink white
  EXPLODE — dramatic explosion sound + LED, then freezes
  FROZEN  — waits for ultrasonic reset (hold hand ~0cm from sensor)

LED  : 4x white LEDs on GPIO14
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

# ──────────────────────────────────────────────────────────────
# STATE CONSTANTS
# ──────────────────────────────────────────────────────────────
IDLE    = "IDLE"
CHASING = "CHASING"
PRIMED  = "PRIMED"
EXPLODE = "EXPLODE"
FROZEN  = "FROZEN"

# ──────────────────────────────────────────────────────────────
# SHARED STATE
# Written by background threads, read by the main loop.
# motor_command is a tuple (method_name, [args]) consumed by
# motor_thread. The state machine sets it and returns immediately
# — the motor thread handles all timing and execution.
# ──────────────────────────────────────────────────────────────
state               = IDLE
person_detected     = False
new_frame_available = False
person_offset_x     = 0.0
dist_front          = None
dist_back           = None
motor_command       = ("stop", [])

# ──────────────────────────────────────────────────────────────
# GPIO INIT
# ──────────────────────────────────────────────────────────────
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

def log(msg):
    with open("/home/creepah/creeper_log.txt", "a") as f:
        f.write(f"{time.strftime('%H:%M:%S')} {msg}\n")


# ──────────────────────────────────────────────────────────────
# LED CONTROLLER
# ──────────────────────────────────────────────────────────────
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

    def cleanup(self):
        self.off()
        self.pwm.stop()


# ──────────────────────────────────────────────────────────────
# BUZZER CONTROLLER
# ──────────────────────────────────────────────────────────────
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


# ──────────────────────────────────────────────────────────────
# MOTOR CONTROLLER
# ──────────────────────────────────────────────────────────────
class MotorController:
    def __init__(self):
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

        self.pwm_fr = GPIO.PWM(config.MOTOR_F_EnA, config.MOTOR_PWM_FREQ)
        self.pwm_fl = GPIO.PWM(config.MOTOR_F_EnB, config.MOTOR_PWM_FREQ)
        self.pwm_br = GPIO.PWM(config.MOTOR_B_EnA, config.MOTOR_PWM_FREQ)
        self.pwm_bl = GPIO.PWM(config.MOTOR_B_EnB, config.MOTOR_PWM_FREQ)
        for pwm in [self.pwm_fl, self.pwm_fr, self.pwm_br, self.pwm_bl]:
            pwm.start(0)
            time.sleep(0.05)

    def _set_left(self, fwd, speed):
        GPIO.output(config.MOTOR_F_3, GPIO.LOW  if fwd else GPIO.HIGH)
        GPIO.output(config.MOTOR_F_4, GPIO.HIGH if fwd else GPIO.LOW)
        GPIO.output(config.MOTOR_B_3, GPIO.LOW  if fwd else GPIO.HIGH)
        GPIO.output(config.MOTOR_B_4, GPIO.HIGH if fwd else GPIO.LOW)
        self.pwm_fl.ChangeDutyCycle(speed)
        self.pwm_bl.ChangeDutyCycle(speed)

    def _set_right(self, fwd, speed):
        GPIO.output(config.MOTOR_F_1, GPIO.HIGH if fwd else GPIO.LOW)
        GPIO.output(config.MOTOR_F_2, GPIO.LOW  if fwd else GPIO.HIGH)
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

    def turn_right(self, speed=50):
        self._set_left(False, speed)
        self._set_right(True, speed)

    def turn_left(self, speed=50):
        self._set_left(True,  speed)
        self._set_right(False, speed)

    def smooth_right(self, speed=50):
        self._set_right(True,  0)
        self._set_left(True, speed)

    def smooth_left(self, speed=50):
        self._set_right(True,  speed)
        self._set_left(True, 0)

    def steer(self, offset_x, base_speed=55):
        if abs(offset_x) < config.DEAD_ZONE:
            self.forward(base_speed)
            return
        if offset_x > 0:
            self.smooth_right(base_speed)
        else:
            self.smooth_left(base_speed)

    def cleanup(self):
        self.stop()
        for pwm in [self.pwm_fl, self.pwm_fr, self.pwm_bl, self.pwm_br]:
            pwm.stop()


# ──────────────────────────────────────────────────────────────
# MOTOR THREAD
# The motor thread is the only place that drives the motors.
# It reads motor_command every 50ms and executes it.
#
# Special command: ("idle_wander", [])
# Instead of a simple method call, this runs the full wander
# sequence internally — obstacle check, ramp-up, timed hold,
# and mid-move obstacle recheck — all without blocking the
# state machine. The wander sequence also checks whether the
# state has changed (e.g. person detected) and exits early
# if the state machine has moved on.
# ──────────────────────────────────────────────────────────────
RESTART_DISTANCE_CM = 3

def _do_idle_wander(mc):
    """
    Full idle wander sequence — runs inside motor_thread.
    Picks a random action, ramps up, holds for a random duration,
    and checks for obstacles mid-move. Exits early if state
    changes away from IDLE so the state machine stays responsive.
    """
    global motor_command

    front_blocked = dist_front is not None and dist_front < config.IDLE_OBSTACLE_CM
    back_blocked  = dist_back  is not None and dist_back  < config.IDLE_OBSTACLE_CM

    if random.uniform(0.0, 1.0) < config.NO_MOTION_CHANCE:
        action = "still"
        duration = random.uniform(1.0, 3.0)
    else: 
        action   = random.choice(["forward", "turn_left", "turn_right", "backward"])
        duration = random.uniform(0.5, 2.0)
        
        if front_blocked:
            action = "backward"
        elif action == "backward" and back_blocked:
            action = random.choice(["forward", "turn_left", "turn_right"])

    log(f"Idle: {action} for {duration:.1f}s")
    print(f"[Motor] Idle: {action} for {duration:.1f}s | front={dist_front}cm back={dist_back}cm")

    # Stop briefly before changing direction
    mc.stop()
    time.sleep(0.1)

    # Ramp up
    if action == "forward":
        for speed in range(10, 26, 5):
            if state != IDLE: mc.stop(); return
            mc.forward(speed)
            time.sleep(0.05)
    elif action == "turn_left":
        for speed in range(20, 70, 5):
            if state != IDLE: mc.stop(); return
            mc.smooth_left(speed)
            duration = duration * 1.5 # longer turn duration
            time.sleep(0.05)
    elif action == "turn_right":
        for speed in range(20, 70, 5):
            if state != IDLE: mc.stop(); return
            mc.smooth_right(speed)
            duration = duration * 1.5 # longer turn duration
            time.sleep(0.05)
    elif action == "backward":
        for speed in range(10, 26, 5):
            if state != IDLE: mc.stop(); return
            mc.backward(speed)
            time.sleep(0.05)
    else:
        mc.stop()
        return

    # Hold for duration, checking obstacles and state changes
    deadline = time.time() + duration
    while time.time() < deadline:
        if state != IDLE:
            mc.stop()
            return
        if dist_front is not None and dist_front < config.IDLE_OBSTACLE_CM and action == "forward":
            mc.stop()
            return
        if dist_back is not None and dist_back < config.IDLE_OBSTACLE_CM and action == "backward":
            mc.stop()
            return
        time.sleep(0.05)

    mc.stop()


def motor_thread(mc):
    """
    Reads motor_command and executes it.
    idle_wander is handled as a full blocking sequence internally.
    All other commands are simple method calls on MotorController.
    """
    try:
        while True:
            cmd, args = motor_command
            try:
                if cmd == "idle_wander":
                    _do_idle_wander(mc)
                else:
                    getattr(mc, cmd)(*args)
            except Exception as e:
                print(f"[Motor] Command error ({cmd} {args}): {e}")
            time.sleep(0.05)
    except Exception as e:
        print(f"[Motor] THREAD CRASHED: {e}")


# ──────────────────────────────────────────────────────────────
# ULTRASONIC SENSOR
# ──────────────────────────────────────────────────────────────
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

def _averaged_distance(echo_pin, samples=5):
    readings = []
    for _ in range(samples):
        dist = _measure_distance(echo_pin)
        if dist is not None:
            readings.append(dist)
        time.sleep(0.01)
    if not readings:
        return None
    readings.sort()
    return readings[len(readings) // 2]

def ultrasonic_thread():
    global dist_front, dist_back
    try:
        GPIO.setup(config.TRIG_PIN,       GPIO.OUT)
        GPIO.setup(config.ECHO_PIN_FRONT, GPIO.IN)
        GPIO.setup(config.ECHO_PIN_BACK,  GPIO.IN)
        while True:
            dist_front = _averaged_distance(config.ECHO_PIN_FRONT, samples=5)
            time.sleep(0.01)
            dist_back  = _averaged_distance(config.ECHO_PIN_BACK,  samples=5)
            time.sleep(0.05)
    except Exception as e:
        print(f"[Ultrasonic] THREAD CRASHED: {e}")


# ──────────────────────────────────────────────────────────────
# VISION
# ──────────────────────────────────────────────────────────────
FRAME_SKIP = 5

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
    global person_detected, person_offset_x, new_frame_available
    try:
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
        frame_cx     = config.CAMERA_WIDTH / 2
        skip_counter = 0
        boxes        = []

        while True:
            ret, frame = cap.read()
            if not ret:
                continue
            skip_counter += 1
            if skip_counter >= FRAME_SKIP:
                skip_counter = 0
                boxes = detect_people(session, input_name, frame)
                new_frame_available = True
            if boxes:
                x1, y1, x2, y2, conf = boxes[0]
                person_detected = True
                person_offset_x = ((x1 + x2) / 2) - frame_cx
            else:
                person_detected = False
                person_offset_x = 0.0
            log(f"Vision: person={person_detected} offset={person_offset_x:.0f}")
            time.sleep(0.1)
    except Exception as e:
        print(f"[Vision] THREAD CRASHED: {e}")


# ──────────────────────────────────────────────────────────────
# RESTART CHECK
# ──────────────────────────────────────────────────────────────
def check_restart():
    if dist_front is not None and dist_front < RESTART_DISTANCE_CM:
        return True
    if dist_back is not None and dist_back < RESTART_DISTANCE_CM:
        return True
    return False


# ──────────────────────────────────────────────────────────────
# MAIN LOOP
# The state machine sets motor_command and returns immediately.
# All motor execution — including the full idle wander sequence
# — happens in motor_thread without blocking the state checker.
#
# IDLE    -> sets ("idle_wander", []), checks for person
# CHASING -> sets ("steer", [...]), checks distance
# PRIMED  -> sets ("stop", []), plays hiss and blink
# EXPLODE -> sets ("stop", []), plays explosion
# FROZEN  -> sets ("stop", []), waits for sensor touch
# ──────────────────────────────────────────────────────────────
def main():
    global state, new_frame_available, motor_command

    mc     = MotorController()
    led    = LEDController()
    buzzer = BuzzerController()

    print("[Creeper] Starting threads...")
    threading.Thread(target=ultrasonic_thread,        daemon=True).start()
    threading.Thread(target=vision_thread,            daemon=True).start()
    threading.Thread(target=motor_thread, args=(mc,), daemon=True).start()

    print("[Creeper] Warming up...")
    time.sleep(2)
    motor_command = ("stop", [])
    time.sleep(0.5)
    print("[Creeper] Running. State: IDLE")

    try:
        while True:
            try:
                log(f"State: {state}")

                # ── IDLE ──────────────────────────────────────────
                # Sets idle_wander command and returns immediately.
                # motor_thread runs the full wander sequence and
                # exits early if the state changes.
                if state == IDLE:
                    led.off()
                    buzzer.off()
                    motor_command = ("idle_wander", [])
                    # Yield briefly so motor_thread can start the wander
                    time.sleep(0.1)
                    print("[Creeper] Scanning for people...")
                    if person_detected:
                        print("[Creeper] Person detected — CHASING")
                        motor_command = ("stop", [])
                        led.flash(duration=0.3, brightness=100)
                        state = CHASING

                # ── CHASING ───────────────────────────────────────
                elif state == CHASING:
                    if not person_detected:
                        print("[Creeper] Lost person — IDLE")
                        motor_command = ("stop", [])
                        state = IDLE
                        continue

                    if dist_front is not None and dist_front <= 50 and abs(person_offset_x) < config.DEAD_ZONE:
                        print("[Creeper] Person within 50cm — PRIMED")
                        motor_command = ("stop", [])
                        state = PRIMED
                        continue

                    if new_frame_available:
                        new_frame_available = False
                        log(f"Chasing: offset={person_offset_x:.0f}px front={dist_front}cm back={dist_back}cm")
                        print(f"[Creeper] Chasing: offset={person_offset_x:.0f}px")
                        motor_command = ("steer", [person_offset_x, 70])
                    time.sleep(0.05)

                # ── PRIMED ────────────────────────────────────────
                elif state == PRIMED:
                    motor_command = ("stop", [])
                    print("[Creeper] Hissing...")
                    dist = dist_front if dist_front is not None else 50
                    buzzer.play_creeper_hiss(dist, led)
                    time.sleep(0.2)
                    led.blink_white(duration=1.5, interval=0.2)
                    print("[Creeper] EXPLODING")
                    state = EXPLODE

                # ── EXPLODE ───────────────────────────────────────
                elif state == EXPLODE:
                    motor_command = ("stop", [])
                    buzzer.play_explosion(led)
                    print("[Creeper] Frozen. Touch a sensor to restart.")
                    state = FROZEN

                # ── FROZEN ────────────────────────────────────────
                elif state == FROZEN:
                    motor_command = ("stop", [])
                    led.off()
                    buzzer.off()
                    if check_restart():
                        print("[Creeper] Restart triggered — IDLE")
                        time.sleep(1)
                        state = IDLE
                    else:
                        time.sleep(0.1)

            except Exception as e:
                print(f"[Main] Exception in state {state}: {e}")
                motor_command = ("stop", [])
                time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n[Creeper] Shutting down.")
    finally:
        motor_command = ("stop", [])
        time.sleep(0.1)
        mc.cleanup()
        led.cleanup()
        buzzer.cleanup()
        GPIO.cleanup()


if __name__ == "__main__":
    main()