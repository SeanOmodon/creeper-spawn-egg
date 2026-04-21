"""
Proximity Alert — LED + Buzzer
Blinks LED and beeps buzzer when person is within ALERT_DISTANCE_CM.
Rate increases as person gets closer.
Run: python3 proximity_alert.py
"""

import time, sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
import config
import RPi.GPIO as GPIO

# ── Setup ──────────────────────────────────────────────────────
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# Ultrasonic
GPIO.setup(config.TRIG_PIN,       GPIO.OUT)
GPIO.setup(config.ECHO_PIN_FRONT, GPIO.IN)

# LED
GPIO.setup(config.LED_R, GPIO.OUT)
GPIO.setup(config.LED_G, GPIO.OUT)
GPIO.setup(config.LED_B, GPIO.OUT)

# Buzzer
GPIO.setup(config.BUZZER_PIN, GPIO.OUT)
GPIO.output(config.BUZZER_PIN, GPIO.LOW)

# Software PWM for LED and buzzer
pwm_r      = GPIO.PWM(config.LED_R,      200)
pwm_g      = GPIO.PWM(config.LED_G,      200)
pwm_b      = GPIO.PWM(config.LED_B,      200)
pwm_buzzer = GPIO.PWM(config.BUZZER_PIN, 1000)  # 1kHz tone

pwm_r.start(0)
pwm_g.start(0)
pwm_b.start(0)
pwm_buzzer.start(0)

# ── Helpers ────────────────────────────────────────────────────
def measure_distance(timeout=0.04):
    GPIO.output(config.TRIG_PIN, False)
    time.sleep(0.002)
    GPIO.output(config.TRIG_PIN, True)
    time.sleep(0.00001)
    GPIO.output(config.TRIG_PIN, False)

    deadline    = time.time() + timeout
    pulse_start = time.time()
    while GPIO.input(config.ECHO_PIN_FRONT) == 0:
        pulse_start = time.time()
        if time.time() > deadline:
            return None

    deadline  = time.time() + timeout
    pulse_end = pulse_start
    while GPIO.input(config.ECHO_PIN_FRONT) == 1:
        pulse_end = time.time()
        if time.time() > deadline:
            return None

    return (pulse_end - pulse_start) * 17150


def get_interval(distance_cm):
    """
    Maps distance to blink interval.
    At ALERT_DISTANCE_CM -> ALERT_MAX_INTERVAL (slow)
    At 5cm or closer    -> ALERT_MIN_INTERVAL (fast)
    """
    min_dist = 5.0
    max_dist = float(config.ALERT_DISTANCE_CM)
    dist     = max(min_dist, min(distance_cm, max_dist))

    # Linear interpolation — closer = shorter interval
    ratio    = (max_dist - dist) / (max_dist - min_dist)
    interval = config.ALERT_MAX_INTERVAL - ratio * (config.ALERT_MAX_INTERVAL - config.ALERT_MIN_INTERVAL)
    return interval


def led_on(r, g, b):
    pwm_r.ChangeDutyCycle(r)
    pwm_g.ChangeDutyCycle(g)
    pwm_b.ChangeDutyCycle(b)


def led_off():
    pwm_r.ChangeDutyCycle(0)
    pwm_g.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0)


def buzzer_on():
    pwm_buzzer.ChangeDutyCycle(50)  # 50% duty = solid tone


def buzzer_off():
    pwm_buzzer.ChangeDutyCycle(0)


def alert_off():
    led_off()
    buzzer_off()


# ── Main loop ──────────────────────────────────────────────────
def main():
    print(f"[Alert] Running. Trigger distance: {config.ALERT_DISTANCE_CM}cm")
    print("[Alert] Ctrl+C to stop")

    led_state = False   # track whether LED/buzzer are currently on

    try:
        while True:
            dist = measure_distance()

            if dist is None:
                print("[Alert] Sensor timeout — check wiring")
                alert_off()
                time.sleep(0.2)
                continue

            print(f"[Alert] Distance: {dist:.1f} cm", end="\r")

            if dist <= config.ALERT_DISTANCE_CM:
                interval = get_interval(dist)
                half     = interval / 2

                # Flash ON — red LED + buzzer
                led_on(100, 0, 0)
                buzzer_on()
                time.sleep(half)

                # Flash OFF
                alert_off()
                time.sleep(half)

            else:
                # Out of range — make sure everything is off
                alert_off()
                time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n[Alert] Shutting down.")
    finally:
        alert_off()
        pwm_r.stop()
        pwm_g.stop()
        pwm_b.stop()
        pwm_buzzer.stop()
        GPIO.cleanup()


if __name__ == "__main__":
    main()