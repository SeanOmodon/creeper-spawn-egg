"""
Stage 4 — Motor control test (GPIO)
Run: python3 tests/04_motor_control.py
Pass condition: motors execute the sequence without GPIO errors.
Requires: L298N or similar H-bridge wired to pins in config.py
"""

import time, sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
import config
import pigpio

class MotorController:
    def __init__(self):
        self.pi = pigpio.pi()

    def forward(self, speed=60):
        self.pi.hardware_PWM(config.MOTOR_FL_EnB, config.MOTOR_PWM_FREQ, int(speed * 10000))
        self.pi.hardware_PWM(config.MOTOR_FR_EnA, config.MOTOR_PWM_FREQ, int(speed * 10000))

    def stop(self):
        self.pi.hardware_PWM(config.MOTOR_FL_EnB, config.MOTOR_PWM_FREQ, 0)
        self.pi.hardware_PWM(config.MOTOR_FR_EnA, config.MOTOR_PWM_FREQ, 0)

    def turn_left(self, speed=50):
        self.pi.hardware_PWM(config.MOTOR_FL_EnB, config.MOTOR_PWM_FREQ, 0)
        self.pi.hardware_PWM(config.MOTOR_FR_EnA, config.MOTOR_PWM_FREQ, int(speed * 10000))

    def turn_right(self, speed=50):
        self.pi.hardware_PWM(config.MOTOR_FL_EnB, config.MOTOR_PWM_FREQ, int(speed * 10000))
        self.pi.hardware_PWM(config.MOTOR_FR_EnA, config.MOTOR_PWM_FREQ, 0)

    def cleanup(self):
        self.stop()
        self.pi.stop()

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