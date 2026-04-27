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
# Five states the creeper cycles through. The main loop checks
# the current state every iteration and decides what to do.
# ──────────────────────────────────────────────────────────────
IDLE    = "IDLE"
CHASING = "CHASING"
PRIMED  = "PRIMED"
EXPLODE = "EXPLODE"
FROZEN  = "FROZEN"

# ──────────────────────────────────────────────────────────────
# SHARED STATE
# These variables are written by background threads (vision,
# ultrasonic) and read by the main control loop. Python's GIL
# makes simple reads/writes safe here without locks.
# ──────────────────────────────────────────────────────────────
state           = IDLE
person_detected = False
person_offset_x = 0.0   # px from frame centre — negative=left, positive=right
dist_front      = None  # cm from front ultrasonic sensor
dist_back       = None  # cm from back ultrasonic sensor

# ──────────────────────────────────────────────────────────────
# GPIO INIT
# Called once at the top so all classes share the same mode.
# setwarnings(False) suppresses "channel already in use" noise
# from previous runs that didn't clean up.
# ──────────────────────────────────────────────────────────────
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

def log(msg):
    with open("/home/creepah/creeper_log.txt", "a") as f:
        f.write(f"{time.strftime('%H:%M:%S')} {msg}\n")

# ──────────────────────────────────────────────────────────────
# LED CONTROLLER
# Controls the 4x white LEDs on GPIO14 via software PWM.
# PWM frequency is 200Hz — fast enough to avoid flicker.
# Brightness is 0–100 duty cycle (0=off, 100=full brightness).
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
        """Single flash — used when creeper first spots a person."""
        self.on(brightness)
        time.sleep(duration)
        self.off()

    def blink_white(self, duration=1.5, interval=0.2):
        """Rapid blinking for the PRIMED countdown."""
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
# Controls the passive buzzer on GPIO20 via software PWM.
# Unlike an active buzzer, a passive buzzer needs PWM to make
# sound — the frequency of the PWM controls the pitch.
# This lets us play specific notes and sweep between frequencies
# to create the creeper hiss and explosion sound effects.
# ──────────────────────────────────────────────────────────────
class BuzzerController:
    def __init__(self):
        GPIO.setup(config.BUZZER_PIN, GPIO.OUT)
        GPIO.output(config.BUZZER_PIN, GPIO.LOW)
        self.pwm = GPIO.PWM(config.BUZZER_PIN, 100)
        self.pwm.start(0)

    def tone(self, freq, duty=40):
        """Play a tone at the given frequency (Hz). duty=40 is a good volume."""
        self.pwm.ChangeFrequency(max(50, freq))  # minimum 50Hz to avoid damage
        self.pwm.ChangeDutyCycle(duty)

    def off(self):
        self.pwm.ChangeDutyCycle(0)

    def play_hiss_burst(self, start_freq, end_freq, duration, steps, led, brightness):
        """
        One descending frequency sweep — sounds like a single 'ssss'.
        Sweeps from start_freq down to end_freq over the given duration.
        LED is held on at the given brightness during the burst.
        """
        step_time = duration / steps
        for i in range(steps):
            freq = int(start_freq + (end_freq - start_freq) * (i / steps))
            self.tone(freq)
            led.on(brightness)
            time.sleep(step_time)
        self.off()
        led.off()

    def play_creeper_hiss(self, dist, led):
        """
        Full Minecraft creeper hiss — multiple descending bursts.
        The closer the person, the more bursts, faster speed,
        brighter LED, and shorter gaps between bursts.
        At 50cm: 1 quiet slow burst.
        At 5cm:  4 loud fast bursts with almost no gap.
        """
        min_dist = 5.0
        max_dist = 50.0
        dist     = max(min_dist, min(dist, max_dist))
        ratio    = (max_dist - dist) / (max_dist - min_dist)  # 0=far, 1=close

        num_bursts = max(1, int(1 + ratio * 3))   # 1 to 4 bursts
        speed      = 1.0 - ratio * 0.6            # 1.0=slow, 0.4=fast
        brightness = int(25 + ratio * 75)          # 25% to 100%
        gap        = 0.12 - ratio * 0.08           # 0.12s to 0.04s between bursts

        stages = [
            (800, 200, 0.18, 30),  # burst 1 — high to low
            (700, 150, 0.15, 25),  # burst 2 — slightly lower
            (600, 100, 0.12, 20),  # burst 3 — lower still
            (500,  80, 0.10, 18),  # burst 4 — deep rumble
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
        """
        Explosion sound — rapid alternating high/low tones with LED flashing,
        dropping in frequency over time like a shockwave dying out.
        """
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
# Controls 4 motors via two L298N drivers using RPi.GPIO
# software PWM on the enable pins.
# Left side  = front-left + back-left motors
# Right side = front-right + back-right motors
# Direction pins (IN1-IN4) set which way each side spins.
# Enable pins (EnA, EnB) control speed via PWM duty cycle 0-100.
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

        self.pwm_fl = GPIO.PWM(config.MOTOR_F_EnA, config.MOTOR_PWM_FREQ)
        self.pwm_fr = GPIO.PWM(config.MOTOR_F_EnB, config.MOTOR_PWM_FREQ)
        self.pwm_br = GPIO.PWM(config.MOTOR_B_EnA, config.MOTOR_PWM_FREQ)
        self.pwm_bl = GPIO.PWM(config.MOTOR_B_EnB, config.MOTOR_PWM_FREQ)
        for pwm in [self.pwm_fl, self.pwm_fr, self.pwm_br, self.pwm_bl]:
            pwm.start(0)
            time.sleep(0.05)

    def _set_left(self, fwd, speed):
        """Drive left side motors forward or backward at given speed."""
        GPIO.output(config.MOTOR_F_1, GPIO.LOW  if fwd else GPIO.HIGH)
        GPIO.output(config.MOTOR_F_2, GPIO.HIGH if fwd else GPIO.LOW)
        GPIO.output(config.MOTOR_B_3, GPIO.LOW  if fwd else GPIO.HIGH)
        GPIO.output(config.MOTOR_B_4, GPIO.HIGH if fwd else GPIO.LOW)
        self.pwm_fl.ChangeDutyCycle(speed)
        self.pwm_bl.ChangeDutyCycle(speed)

    def _set_right(self, fwd, speed):
        """Drive right side motors forward or backward at given speed."""
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

    def turn_right(self, speed=50):
        """Tank turn right — right side backward, left side forward."""
        self._set_left(False, speed)
        self._set_right(True, speed)

    def turn_left(self, speed=50):
        """Tank turn left — left side backward, right side forward."""
        self._set_left(True,  speed)
        self._set_right(False, speed)

    def smooth_left(self, speed=50):
        """Gentle left curve — left side stopped, right side forward."""
        self._set_left(True,  0)
        self._set_right(True, speed)

    def smooth_right(self, speed=50):
        """Gentle right curve — right side stopped, left side forward."""
        self._set_left(True,  speed)
        self._set_right(True, 0)

    def steer(self, offset_x, base_speed=55):
        """
        Smooth steering toward a detected person.
        offset_x is how many pixels the person is from the frame centre.
        Dead zone of 60px drives straight to avoid jitter.
        Outside dead zone: calls smooth_left or smooth_right.
        """
        dead_zone  = 60
        frame_half = config.CAMERA_WIDTH / 2

        if abs(offset_x) < dead_zone:
            self.forward(base_speed)
            return

        if offset_x < 0:
            self.smooth_right(base_speed)
        else:
            self.smooth_left(base_speed)

    def cleanup(self):
        self.stop()
        for pwm in [self.pwm_fl, self.pwm_fr, self.pwm_bl, self.pwm_br]:
            pwm.stop()


# ──────────────────────────────────────────────────────────────
# ULTRASONIC SENSOR
# _measure_distance fires a 10µs trigger pulse and times how
# long the echo pin stays HIGH. Distance = time * speed of sound.
# ultrasonic_thread runs in the background, updating dist_front
# and dist_back every ~60ms. The small sleep between front and
# back readings prevents crosstalk between the two sensors.
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

def _averaged_distance(echo_pin, samples=3):
    """
    Takes multiple readings and returns the median.
    Filters out spurious spikes from electrical noise.
    None readings are ignored — if all readings fail, returns None.
    """
    readings = []
    for _ in range(samples):
        dist = _measure_distance(echo_pin)
        if dist is not None:
            readings.append(dist)
        time.sleep(0.01)

    if not readings:
        return None
    readings.sort()
    return readings[len(readings) // 2]  # median

def ultrasonic_thread():
    global dist_front, dist_back
    try:
        GPIO.setup(config.TRIG_PIN,       GPIO.OUT)
        GPIO.setup(config.ECHO_PIN_FRONT, GPIO.IN)
        GPIO.setup(config.ECHO_PIN_BACK,  GPIO.IN)

        while True:
            dist_front = _averaged_distance(config.ECHO_PIN_FRONT, samples=3)
            time.sleep(0.01)
            dist_back  = _averaged_distance(config.ECHO_PIN_BACK,  samples=3)
            time.sleep(0.05)
    except Exception as e:
        print(f"[Ultrasonic] THREAD CRASHED: {e}")


# ──────────────────────────────────────────────────────────────
# VISION
# load_model loads the YOLOv8n ONNX model from disk.
# detect_people runs inference on a single frame:
#   1. Resize to 320x320 (smaller = faster on Pi)
#   2. Normalise pixel values to 0.0–1.0
#   3. Reshape to (1, 3, 320, 320) — batch of 1, RGB channels
#   4. Run through YOLO
#   5. Filter results to class 0 (person) above confidence floor
#   6. Return bounding boxes scaled back to original resolution
#
# vision_thread runs in the background, updating person_detected
# and person_offset_x every FRAME_SKIP frames. The sleep(0.03)
# gives the CPU breathing room to prevent thermal throttling.
# ──────────────────────────────────────────────────────────────
FRAME_SKIP = 5   # run YOLO every 5th frame to reduce CPU load

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
            if boxes:
                x1, y1, x2, y2, conf = boxes[0]
                person_detected = True
                person_offset_x = ((x1 + x2) / 2) - frame_cx
            else:
                person_detected = False
                person_offset_x = 0.0
            log(f"Vision: person={person_detected} offset={person_offset_x:.0f}")
            time.sleep(0.05)
    except Exception as e:
        print(f"[Vision] THREAD CRASHED: {e}")


# ──────────────────────────────────────────────────────────────
# IDLE WANDERING
# Wanders randomly but checks the front and back ultrasonic
# sensors before each move to avoid driving into obstacles.
# If an obstacle is detected in the intended direction, it
# turns away instead of continuing forward/backward.
# ──────────────────────────────────────────────────────────────

def idle_wander(mc):
    front_blocked = dist_front is not None and dist_front < config.IDLE_OBSTACLE_CM
    back_blocked  = dist_back  is not None and dist_back  < config.IDLE_OBSTACLE_CM

    action   = random.choice(["forward", "turn_left", "turn_right", "backward"])
    duration = random.uniform(0.5, 2.0)

    if front_blocked:
        action = "backward"
    elif action == "backward" and back_blocked:
        action = random.choice(["forward", "turn_left", "turn_right"])

    log(f"Idle: {action} for {duration:.1f}s")
    print(f"[Idle] Action: {action}, Duration: {duration:.1f}s, Front: {dist_front}cm, Back: {dist_back}cm")    

    # Always stop briefly before changing direction — reduces current spike
    mc.stop()
    time.sleep(1)

    # Ramp up speed gradually instead of jumping straight to full speed
    if action == "forward":
        for speed in range(10, 26, 5):
            mc.forward(speed)
            time.sleep(0.05)
    elif action == "turn_left":
        for speed in range(20, 60, 5):
            mc.smooth_left(speed)
            time.sleep(0.05)
    elif action == "turn_right":
        for speed in range(20, 60, 5):
            mc.smooth_right(speed)
            time.sleep(0.05)
    elif action == "backward":
        for speed in range(10, 26, 5):
            mc.backward(speed)
            time.sleep(0.05)
    else:
        mc.stop()

    deadline = time.time() + duration
    while time.time() < deadline:
        if dist_front is not None and dist_front < config.IDLE_OBSTACLE_CM and action == "forward":
            mc.stop()
            break
        if dist_back is not None and dist_back < config.IDLE_OBSTACLE_CM and action == "backward":
            mc.stop()
            break
        time.sleep(0.05)


# ──────────────────────────────────────────────────────────────
# RESTART CHECK
# After exploding the creeper freezes until someone holds their
# hand very close (~0cm) to either ultrasonic sensor.
# RESTART_DISTANCE_CM is set low enough to require deliberate
# contact rather than just walking past.
# ──────────────────────────────────────────────────────────────
RESTART_DISTANCE_CM = 3

def check_restart():
    if dist_front is not None and dist_front < RESTART_DISTANCE_CM:
        return True
    if dist_back is not None and dist_back < RESTART_DISTANCE_CM:
        return True
    return False


# ──────────────────────────────────────────────────────────────
# MAIN LOOP
# Initialises all hardware, starts background threads, then
# runs the state machine in a tight loop.
#
# IDLE    → wanders randomly, checks for person each cycle
# CHASING → steers toward person, checks distance each cycle
# PRIMED  → stops, plays hiss, blinks, then transitions
# EXPLODE → plays explosion sequence, then transitions
# FROZEN  → waits for sensor touch to restart
#
# Ctrl+C triggers cleanup of all GPIO and PWM resources.
# ──────────────────────────────────────────────────────────────
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
    mc.stop()
    time.sleep(0.5)
    print("[Creeper] Running. State: IDLE")

    try:
        while True:
            try:
                log(f"State: {state}")
                # ── IDLE ──────────────────────────────────────────
                if state == IDLE:
                    led.off()
                    buzzer.off()
                    idle_wander(mc)
                    print("[Creeper] Scanning for people...")
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

                    mc.steer(person_offset_x, base_speed=70)
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
            except Exception as e:
                print(f"[Main] Exception in state {state}: {e}")
                mc.stop()
                time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n[Creeper] Shutting down.")
    finally:
        mc.cleanup()
        led.cleanup()
        buzzer.cleanup()
        GPIO.cleanup()


if __name__ == "__main__":
    main()