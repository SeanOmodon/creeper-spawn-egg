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
        pins = [config.MOTOR_LEFT_FWD, config.MOTOR_LEFT_BWD,
                config.MOTOR_RIGHT_FWD, config.MOTOR_RIGHT_BWD]
        for p in pins:
            GPIO.setup(p, GPIO.OUT)

        self.pwm_lf = GPIO.PWM(config.MOTOR_LEFT_FWD,  config.MOTOR_PWM_FREQ)
        self.pwm_lb = GPIO.PWM(config.MOTOR_LEFT_BWD,  config.MOTOR_PWM_FREQ)
        self.pwm_rf = GPIO.PWM(config.MOTOR_RIGHT_FWD, config.MOTOR_PWM_FREQ)
        self.pwm_rb = GPIO.PWM(config.MOTOR_RIGHT_BWD, config.MOTOR_PWM_FREQ)
        for pwm in [self.pwm_lf, self.pwm_lb, self.pwm_rf, self.pwm_rb]:
            pwm.start(0)

    def forward(self, speed=60):
        self.pwm_lf.ChangeDutyCycle(speed); self.pwm_lb.ChangeDutyCycle(0)
        self.pwm_rf.ChangeDutyCycle(speed); self.pwm_rb.ChangeDutyCycle(0)

    def stop(self):
        for pwm in [self.pwm_lf, self.pwm_lb, self.pwm_rf, self.pwm_rb]:
            pwm.ChangeDutyCycle(0)

    def turn_left(self, speed=50):
        self.pwm_lf.ChangeDutyCycle(0);     self.pwm_lb.ChangeDutyCycle(speed)
        self.pwm_rf.ChangeDutyCycle(speed); self.pwm_rb.ChangeDutyCycle(0)

    def turn_right(self, speed=50):
        self.pwm_lf.ChangeDutyCycle(speed); self.pwm_lb.ChangeDutyCycle(0)
        self.pwm_rf.ChangeDutyCycle(0);     self.pwm_rb.ChangeDutyCycle(speed)

    def cleanup(self):
        self.stop()
        GPIO.cleanup()

def main():
    mc = MotorController()
    try:
        sequence = [
            ("Forward",     lambda: mc.forward(60),     1.5),
            ("Stop",        lambda: mc.stop(),           0.5),
            ("Turn left",   lambda: mc.turn_left(50),   1.0),
            ("Stop",        lambda: mc.stop(),           0.5),
            ("Turn right",  lambda: mc.turn_right(50),  1.0),
            ("Stop",        lambda: mc.stop(),           0.5),
        ]
        for label, action, duration in sequence:
            print(f"[Stage 3] {label} for {duration}s...")
            action()
            time.sleep(duration)
        print("[Stage 4] PASS")
    except Exception as e:
        print(f"[Stage 4] FAIL — {e}")
    finally:
        mc.cleanup()

if __name__ == "__main__":
    main()