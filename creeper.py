"""
creeper.py — Main runtime for the Creeper robot
Run: 
sudo pigpiod
python3 creeper.py

States:
  IDLE    — wanders randomly, scanning for a person
  CHASING — moves toward detected person, smooth turning
  PRIMED  — person within 50cm, blinks white for 1.5s
  EXPLODE — flashes red/white rapidly, then freezes
  FROZEN  — waits for ultrasonic reset (distance ~0cm)

LED: RGB strip — R=GPIO15, G=GPIO14, B=GPIO16
Restart: hold hand ~0cm from either ultrasonic sensor
"""

import time
import sys
import os
import threading
import random

sys.path.insert(0, os.path.dirname(__file__))
import config

from picamera2 import Picamera2
import onnxruntime as ort
import numpy as np
import cv2
import pigpio
import RPi.GPIO as GPIO

# ── State constants ────────────────────────────────────────────
IDLE    = "IDLE"
CHASING = "CHASING"
PRIMED  = "PRIMED"
EXPLODE = "EXPLODE"
FROZEN  = "FROZEN"

# ── Shared state ───────────────────────────────────────────────
state            = IDLE
person_detected  = False
person_offset_x  = 0.0      # px from frame centre, negative=left
dist_front       = None     # cm, None = no reading
dist_back        = None     # cm, None = no reading
state_lock       = threading.Lock()

# ── LED ────────────────────────────────────────────────────────
class LEDController:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        for pin in [config.LED_R, config.LED_G, config.LED_B]:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)

        self.pwm_r = GPIO.PWM(config.LED_R, config.LED_PWM_FREQ)
        self.pwm_g = GPIO.PWM(config.LED_G, config.LED_PWM_FREQ)
        self.pwm_b = GPIO.PWM(config.LED_B, config.LED_PWM_FREQ)
        self.pwm_r.start(0)
        self.pwm_g.start(0)
        self.pwm_b.start(0)

    def _set(self, r, g, b):
        """r/g/b are 0–100 duty cycle."""
        self.pwm_r.ChangeDutyCycle(r)
        self.pwm_g.ChangeDutyCycle(g)
        self.pwm_b.ChangeDutyCycle(b)

    def off(self):       self._set(0,   0,   0)
    def red(self):       self._set(100, 0,   0)
    def white(self):     self._set(100, 100, 100)

    def flash_red(self, duration=0.3):
        self.red()
        time.sleep(duration)
        self.off()

    def blink_white(self, duration=1.5, interval=0.2):
        deadline = time.time() + duration
        while time.time() < deadline:
            self.white()
            time.sleep(interval / 2)
            self.off()
            time.sleep(interval / 2)

    def explode(self, flashes=16, interval=0.08):
        for i in range(flashes):
            if i % 2 == 0:
                self.red()
            else:
                self.white()
            time.sleep(interval)
        self.off()

    def cleanup(self):
        self.off()
        self.pwm_r.stop()
        self.pwm_g.stop()
        self.pwm_b.stop()


# ── Motor controller ───────────────────────────────────────────
class MotorController:
    def __init__(self, pi):
        self.pi = pi
        direction_pins = [
            config.MOTOR_FL_1, config.MOTOR_FL_2,
            config.MOTOR_FR_1, config.MOTOR_FR_2,
            config.MOTOR_BR_1, config.MOTOR_BR_2,
            config.MOTOR_BL_1, config.MOTOR_BL_2,
        ]
        for pin in direction_pins:
            pi.set_mode(pin, pigpio.OUTPUT)
            pi.write(pin, 0)

    def _set_left(self, fwd, speed):
        self.pi.write(config.MOTOR_FL_1, 1 if fwd else 0)
        self.pi.write(config.MOTOR_FL_2, 0 if fwd else 1)
        self.pi.write(config.MOTOR_BL_1, 1 if fwd else 0)
        self.pi.write(config.MOTOR_BL_2, 0 if fwd else 1)
        duty = int(speed * 10000)
        self.pi.hardware_PWM(config.MOTOR_FL_EnB, config.MOTOR_PWM_FREQ, duty)
        self.pi.hardware_PWM(config.MOTOR_BL_EnA, config.MOTOR_PWM_FREQ, duty)

    def _set_right(self, fwd, speed):
        self.pi.write(config.MOTOR_FR_1, 1 if fwd else 0)
        self.pi.write(config.MOTOR_FR_2, 0 if fwd else 1)
        self.pi.write(config.MOTOR_BR_1, 1 if fwd else 0)
        self.pi.write(config.MOTOR_BR_2, 0 if fwd else 1)
        duty = int(speed * 10000)
        self.pi.hardware_PWM(config.MOTOR_FR_EnA, config.MOTOR_PWM_FREQ, duty)
        self.pi.hardware_PWM(config.MOTOR_BR_EnB, config.MOTOR_PWM_FREQ, duty)

    def forward(self, speed=60):
        self._set_left(True,  speed)
        self._set_right(True, speed)

    def backward(self, speed=60):
        self._set_left(False,  speed)
        self._set_right(False, speed)

    def stop(self):
        self.pi.hardware_PWM(config.MOTOR_FL_EnB, config.MOTOR_PWM_FREQ, 0)
        self.pi.hardware_PWM(config.MOTOR_FR_EnA, config.MOTOR_PWM_FREQ, 0)
        self.pi.hardware_PWM(config.MOTOR_BL_EnA, config.MOTOR_PWM_FREQ, 0)
        self.pi.hardware_PWM(config.MOTOR_BR_EnB, config.MOTOR_PWM_FREQ, 0)

    def turn_left(self, speed=50):
        self._set_left(False, speed)
        self._set_right(True, speed)

    def turn_right(self, speed=50):
        self._set_left(True,  speed)
        self._set_right(False, speed)

    def steer(self, offset_x, base_speed=55):
        """
        Smooth steering based on pixel offset from centre.
        Negative offset = person is left  -> slow left side
        Positive offset = person is right -> slow right side
        Dead zone of 60px drives straight.
        """
        frame_half  = config.CAMERA_WIDTH / 2   # 320px

        if abs(offset_x) < config.DEAD_ZONE:
            self.forward(base_speed)
            return

        # Scale offset to 0.0–1.0 beyond dead zone
        beyond    = abs(offset_x) - config.DEAD_ZONE
        max_range = frame_half - config.DEAD_ZONE
        factor    = min(beyond / max_range, 1.0)

        # Slow side gets between base_speed and base_speed * 0.2
        slow_speed = base_speed * (1.0 - 0.8 * factor)

        if offset_x < 0:   # person is left
            self._set_left(True,  slow_speed)
            self._set_right(True, base_speed)
        else:               # person is right
            self._set_left(True,  base_speed)
            self._set_right(True, slow_speed)

    def cleanup(self):
        self.stop()


# ── Ultrasonic ─────────────────────────────────────────────────
def _measure_distance(echo_pin, timeout=0.04):
    GPIO.output(config.TRIG_PIN, False)
    time.sleep(0.002)
    GPIO.output(config.TRIG_PIN, True)
    time.sleep(0.00001)
    GPIO.output(config.TRIG_PIN, False)

    deadline = time.time() + timeout
    pulse_start = time.time()
    while GPIO.input(echo_pin) == 0:
        pulse_start = time.time()
        if time.time() > deadline:
            return None

    deadline = time.time() + timeout
    pulse_end = pulse_start
    while GPIO.input(echo_pin) == 1:
        pulse_end = time.time()
        if time.time() > deadline:
            return None

    return (pulse_end - pulse_start) * 17150


def ultrasonic_thread():
    global dist_front, dist_back
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(config.TRIG_PIN,        GPIO.OUT)
    GPIO.setup(config.ECHO_PIN_FRONT,  GPIO.IN)
    GPIO.setup(config.ECHO_PIN_BACK,   GPIO.IN)

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
    session, input_name = load_model()

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
        bgr   = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        boxes = detect_people(session, input_name, bgr)

        if boxes:
            x1, y1, x2, y2, conf = boxes[0]
            person_detected  = True
            person_offset_x  = ((x1 + x2) / 2) - frame_cx
        else:
            person_detected  = False
            person_offset_x  = 0.0


# ── Idle wandering ─────────────────────────────────────────────
def idle_wander(mc):
    """One step of random wandering — called repeatedly in the main loop."""
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
def check_restart():
    """Returns True if either sensor is touched (distance ~0)."""
    if dist_front is not None and dist_front < config.RESTART_DISTANCE_CM:
        return True
    if dist_back is not None and dist_back < config.RESTART_DISTANCE_CM:
        return True
    return False


# ── Main loop ──────────────────────────────────────────────────
def main():
    global state

    print("[Creeper] Starting pigpio...")
    pi = pigpio.pi()
    if not pi.connected:
        print("[Creeper] FAIL — run: sudo pigpiod")
        sys.exit(1)

    mc  = MotorController(pi)
    led = LEDController()

    print("[Creeper] Starting threads...")
    threading.Thread(target=ultrasonic_thread, daemon=True).start()
    threading.Thread(target=vision_thread,     daemon=True).start()

    print("[Creeper] Waiting for sensors to warm up...")
    time.sleep(2)

    print("[Creeper] Running. State: IDLE")

    try:
        while True:

            # ── IDLE ──────────────────────────────────────────
            if state == IDLE:
                led.off()
                idle_wander(mc)

                if person_detected:
                    print("[Creeper] Person detected — CHASING")
                    led.flash_red(duration=0.3)
                    state = CHASING

            # ── CHASING ───────────────────────────────────────
            elif state == CHASING:
                if not person_detected:
                    # Lost the person — go back to wandering
                    print("[Creeper] Lost person — IDLE")
                    mc.stop()
                    state = IDLE
                    continue

                # Check front sensor for explosion trigger
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
                print("[Creeper] Blinking white...")
                led.blink_white(duration=1.5, interval=0.2)
                print("[Creeper] EXPLODING")
                state = EXPLODE

            # ── EXPLODE ───────────────────────────────────────
            elif state == EXPLODE:
                mc.stop()
                led.explode(flashes=16, interval=0.08)
                print("[Creeper] Frozen. Touch a sensor to restart.")
                state = FROZEN

            # ── FROZEN ────────────────────────────────────────
            elif state == FROZEN:
                mc.stop()
                led.off()

                if check_restart():
                    print("[Creeper] Restart triggered — IDLE")
                    time.sleep(1)   # debounce
                    state = IDLE
                else:
                    time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n[Creeper] Shutting down.")
    finally:
        mc.cleanup()
        led.cleanup()
        GPIO.cleanup()
        pi.stop()


if __name__ == "__main__":
    main()