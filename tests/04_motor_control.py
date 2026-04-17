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
        if not self.pi.connected:
            print("[Motors] FAIL — pigpio daemon not running. Start with: sudo pigpiod")
            sys.exit(1)
        direction_pins = [
            config.MOTOR_FL_1, config.MOTOR_FL_2,
            config.MOTOR_FR_1, config.MOTOR_FR_2,
            config.MOTOR_BR_1, config.MOTOR_BR_2,
            config.MOTOR_BL_1, config.MOTOR_BL_2,
        ]
        for pin in direction_pins:
            self.pi.set_mode(pin, pigpio.OUTPUT)
            self.pi.write(pin, 0)

    def forward(self, speed=60):
        # All motors forward
        self.pi.write(config.MOTOR_FL_1, 1); self.pi.write(config.MOTOR_FL_2, 0)
        self.pi.write(config.MOTOR_FR_1, 1); self.pi.write(config.MOTOR_FR_2, 0)
        self.pi.write(config.MOTOR_BL_1, 1); self.pi.write(config.MOTOR_BL_2, 0)
        self.pi.write(config.MOTOR_BR_1, 1); self.pi.write(config.MOTOR_BR_2, 0)
        duty = int(speed * 10000)
        self.pi.hardware_PWM(config.MOTOR_FL_EnB, config.MOTOR_PWM_FREQ, duty)
        self.pi.hardware_PWM(config.MOTOR_FR_EnA, config.MOTOR_PWM_FREQ, duty)
        self.pi.hardware_PWM(config.MOTOR_BL_EnA, config.MOTOR_PWM_FREQ, duty)
        self.pi.hardware_PWM(config.MOTOR_BR_EnB, config.MOTOR_PWM_FREQ, duty)

    def stop(self):
        self.pi.hardware_PWM(config.MOTOR_FL_EnB, config.MOTOR_PWM_FREQ, 0)
        self.pi.hardware_PWM(config.MOTOR_FR_EnA, config.MOTOR_PWM_FREQ, 0)
        self.pi.hardware_PWM(config.MOTOR_BL_EnA, config.MOTOR_PWM_FREQ, 0)
        self.pi.hardware_PWM(config.MOTOR_BR_EnB, config.MOTOR_PWM_FREQ, 0)

    def turn_left(self, speed=50):
        # Left side backward, right side forward
        self.pi.write(config.MOTOR_FL_1, 0); self.pi.write(config.MOTOR_FL_2, 1)
        self.pi.write(config.MOTOR_BL_1, 0); self.pi.write(config.MOTOR_BL_2, 1)
        self.pi.write(config.MOTOR_FR_1, 1); self.pi.write(config.MOTOR_FR_2, 0)
        self.pi.write(config.MOTOR_BR_1, 1); self.pi.write(config.MOTOR_BR_2, 0)
        duty = int(speed * 10000)
        self.pi.hardware_PWM(config.MOTOR_FL_EnB, config.MOTOR_PWM_FREQ, duty)
        self.pi.hardware_PWM(config.MOTOR_FR_EnA, config.MOTOR_PWM_FREQ, duty)
        self.pi.hardware_PWM(config.MOTOR_BL_EnA, config.MOTOR_PWM_FREQ, duty)
        self.pi.hardware_PWM(config.MOTOR_BR_EnB, config.MOTOR_PWM_FREQ, duty)

    def turn_right(self, speed=50):
        # Right side backward, left side forward
        self.pi.write(config.MOTOR_FL_1, 1); self.pi.write(config.MOTOR_FL_2, 0)
        self.pi.write(config.MOTOR_BL_1, 1); self.pi.write(config.MOTOR_BL_2, 0)
        self.pi.write(config.MOTOR_FR_1, 0); self.pi.write(config.MOTOR_FR_2, 1)
        self.pi.write(config.MOTOR_BR_1, 0); self.pi.write(config.MOTOR_BR_2, 1)
        duty = int(speed * 10000)
        self.pi.hardware_PWM(config.MOTOR_FL_EnB, config.MOTOR_PWM_FREQ, duty)
        self.pi.hardware_PWM(config.MOTOR_FR_EnA, config.MOTOR_PWM_FREQ, duty)
        self.pi.hardware_PWM(config.MOTOR_BL_EnA, config.MOTOR_PWM_FREQ, duty)
        self.pi.hardware_PWM(config.MOTOR_BR_EnB, config.MOTOR_PWM_FREQ, duty)

    def smooth_left(self, speed=50):
        # Left side slower, right side forward
        self.pi.write(config.MOTOR_FL_1, 1); self.pi.write(config.MOTOR_FL_2, 0)
        self.pi.write(config.MOTOR_BL_1, 1); self.pi.write(config.MOTOR_BL_2, 0)
        self.pi.write(config.MOTOR_FR_1, 1); self.pi.write(config.MOTOR_FR_2, 0)
        self.pi.write(config.MOTOR_BR_1, 1); self.pi.write(config.MOTOR_BR_2, 0)
        duty_left = int(speed * 10000 * 0.5)   # left motors at half speed
        duty_right = int(speed * 10000)
        self.pi.hardware_PWM(config.MOTOR_FL_EnB, config.MOTOR_PWM_FREQ, duty_left)
        self.pi.hardware_PWM(config.MOTOR_FR_EnA, config.MOTOR_PWM_FREQ, duty_right)
        self.pi.hardware_PWM(config.MOTOR_BL_EnA, config.MOTOR_PWM_FREQ, duty_left)
        self.pi.hardware_PWM(config.MOTOR_BR_EnB, config.MOTOR_PWM_FREQ, duty_right)
    
    def smooth_right(self, speed=50):
        # Right side slower, left side forward
        self.pi.write(config.MOTOR_FL_1, 1); self.pi.write(config.MOTOR_FL_2, 0)
        self.pi.write(config.MOTOR_BL_1, 1); self.pi.write(config.MOTOR_BL_2, 0)
        self.pi.write(config.MOTOR_FR_1, 1); self.pi.write(config.MOTOR_FR_2, 0)
        self.pi.write(config.MOTOR_BR_1, 1); self.pi.write(config.MOTOR_BR_2, 0)
        duty_left = int(speed * 10000)
        duty_right = int(speed * 10000 * 0.5)   # right motors at half speed
        self.pi.hardware_PWM(config.MOTOR_FL_EnB, config.MOTOR_PWM_FREQ, duty_left)
        self.pi.hardware_PWM(config.MOTOR_FR_EnA, config.MOTOR_PWM_FREQ, duty_right)
        self.pi.hardware_PWM(config.MOTOR_BL_EnA, config.MOTOR_PWM_FREQ, duty_left)
        self.pi.hardware_PWM(config.MOTOR_BR_EnB, config.MOTOR_PWM_FREQ, duty_right)

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
            ("Smooth left", lambda: mc.smooth_left(50),  1.0),
            ("Stop",        lambda: mc.stop(),           0.5),
            ("Smooth right", lambda: mc.smooth_right(50),  1.0),
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