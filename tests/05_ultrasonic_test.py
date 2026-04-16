"""
Stage 5 — HC-SR04 ultrasonic sensor test
Run: python3 tests/05_ultrasonic_test.py
Pass condition: stable readings printed for 15s; obstacle threshold triggered correctly.
"""

import time, sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
import config
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setup(config.TRIG_PIN, GPIO.OUT)
GPIO.setup(config.ECHO_PIN, GPIO.IN)

def read_distance_cm(timeout=0.04):
    """Returns distance in cm, or None on timeout."""
    GPIO.output(config.TRIG_PIN, False)
    time.sleep(0.002)

    GPIO.output(config.TRIG_PIN, True)
    time.sleep(0.00001)
    GPIO.output(config.TRIG_PIN, False)

    pulse_start = time.time()
    deadline = pulse_start + timeout
    while GPIO.input(config.ECHO_PIN) == 0:
        pulse_start = time.time()
        if pulse_start > deadline:
            return None

    pulse_end = time.time()
    deadline = pulse_end + timeout
    while GPIO.input(config.ECHO_PIN) == 1:
        pulse_end = time.time()
        if pulse_end > deadline:
            return None

    return round((pulse_end - pulse_start) * 17150, 1)  # cm

def main():
    print(f"[Stage 5] Reading ultrasonic for 15s. Obstacle threshold: {config.OBSTACLE_DISTANCE_CM}cm")
    print("[Stage 5] Move your hand toward the sensor to test threshold trigger.")
    deadline = time.time() + 15
    obstacle_triggers = 0

    try:
        while time.time() < deadline:
            dist = read_distance_cm()
            if dist is None:
                print("[Stage 5] Timeout reading pulse — check wiring")
                time.sleep(0.2)
                continue

            status = "OBSTACLE" if dist < config.OBSTACLE_DISTANCE_CM else "clear"
            if dist < config.OBSTACLE_DISTANCE_CM:
                obstacle_triggers += 1
            print(f"[Stage 5] {dist:6.1f} cm  [{status}]")
            time.sleep(0.1)
    finally:
        GPIO.cleanup()

    print(f"[Stage 5] Obstacle triggers: {obstacle_triggers}")
    print("[Stage 5] PASS" if obstacle_triggers > 0 else "[Stage 5] NOTE — no obstacles detected; verify sensor wiring if unexpected")

if __name__ == "__main__":
    main()