"""
Stage 4 — Motor control test (GPIO)
Run: python3 tests/04_motor_control.py
Pass condition: motors execute the sequence without GPIO errors.
Requires: L298N or similar H-bridge wired to pins in config.py
"""

import time, sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
import config
import RPi.GPIO as GPIO

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
        self.pwm_br = GPIO.PWM(config.MOTOR_B_EnA, config.MOTOR_PWM_FREQ)
        self.pwm_bl = GPIO.PWM(config.MOTOR_B_EnB, config.MOTOR_PWM_FREQ)
        for pwm in [self.pwm_fl, self.pwm_fr, self.pwm_br, self.pwm_bl]:
            pwm.start(0)

    def _set_left(self, fwd, speed):
        GPIO.output(config.MOTOR_F_1, GPIO.LOW if fwd else GPIO.HIGH)
        GPIO.output(config.MOTOR_F_2, GPIO.HIGH  if fwd else GPIO.LOW)
        GPIO.output(config.MOTOR_B_3, GPIO.LOW if fwd else GPIO.HIGH)
        GPIO.output(config.MOTOR_B_4, GPIO.HIGH  if fwd else GPIO.LOW)
        self.pwm_fl.ChangeDutyCycle(speed)
        self.pwm_bl.ChangeDutyCycle(speed)

    def _set_right(self, fwd, speed):
        GPIO.output(config.MOTOR_F_3, GPIO.HIGH if fwd else GPIO.LOW)
        GPIO.output(config.MOTOR_F_4, GPIO.LOW  if fwd else GPIO.HIGH)
        GPIO.output(config.MOTOR_B_1, GPIO.HIGH if fwd else GPIO.LOW)
        GPIO.output(config.MOTOR_B_2, GPIO.LOW  if fwd else GPIO.HIGH)
        self.pwm_fr.ChangeDutyCycle(speed)
        self.pwm_br.ChangeDutyCycle(speed)

    def forward(self, speed=30):
        self._set_left(True,  speed)
        self._set_right(True, speed)

    def backward(self, speed=30):
        self._set_left(False,  speed)
        self._set_right(False, speed)

    def stop(self):
        for pwm in [self.pwm_fl, self.pwm_fr, self.pwm_bl, self.pwm_br]:
            pwm.ChangeDutyCycle(0)

    def turn_left(self, speed=30):
        self._set_left(False, speed)
        self._set_right(True, speed)

    def turn_right(self, speed=30):
        self._set_left(True,  speed)
        self._set_right(False, speed)

    def smooth_left(self, speed=30):
        self._set_left(True,  speed * 0.5)
        self._set_right(True, speed)

    def smooth_right(self, speed=30):
        self._set_left(True,  speed)
        self._set_right(True, speed * 0.5)

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


def main():
    mc = MotorController()
    try:
        sequence = [
            ("Forward",      lambda: mc.forward(30),      3.0),
            ("Stop",         lambda: mc.stop(),            1.0),
            ("Turn left",    lambda: mc.turn_left(30),    3.0),
            ("Stop",         lambda: mc.stop(),            1.0),
            ("Turn right",   lambda: mc.turn_right(30),   3.0),
            ("Stop",         lambda: mc.stop(),            1.0),
            ("Smooth left",  lambda: mc.smooth_left(30),  3.0),
            ("Stop",         lambda: mc.stop(),            1.0),
            ("Smooth right", lambda: mc.smooth_right(30), 3.0),
            ("Stop",         lambda: mc.stop(),            1.0),
        ]
        for label, action, duration in sequence:
            print(f"[Stage 4] {label} for {duration}s...")
            action()
            time.sleep(duration)
        print("[Stage 4] PASS")
    except Exception as e:
        print(f"[Stage 4] FAIL — {e}")
    finally:
        mc.cleanup()

if __name__ == "__main__":
    main()