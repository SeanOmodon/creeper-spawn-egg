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
    def __init__(self, pi):
        self.pi = pi
        all_pins = [
            config.MOTOR_FL_1, config.MOTOR_FL_2,
            config.MOTOR_FR_1, config.MOTOR_FR_2,
            config.MOTOR_BR_1, config.MOTOR_BR_2,
            config.MOTOR_BL_1, config.MOTOR_BL_2,
            config.MOTOR_FL_EnB, config.MOTOR_FR_EnA,
            config.MOTOR_BL_EnA, config.MOTOR_BR_EnB,
        ]
        for pin in all_pins:
            pi.set_mode(pin, pigpio.OUTPUT)
            pi.write(pin, 0)

    def _set_left(self, fwd, speed):
        self.pi.write(config.MOTOR_FL_1, 1 if fwd else 0)
        self.pi.write(config.MOTOR_FL_2, 0 if fwd else 1)
        self.pi.write(config.MOTOR_BL_1, 1 if fwd else 0)
        self.pi.write(config.MOTOR_BL_2, 0 if fwd else 1)
        # Only call hardware_PWM on GPIO 13 — GPIO 19 mirrors it automatically
        self.pi.hardware_PWM(13, config.MOTOR_PWM_FREQ, int(speed * 10000))

    def _set_right(self, fwd, speed):
        self.pi.write(config.MOTOR_FR_1, 1 if fwd else 0)
        self.pi.write(config.MOTOR_FR_2, 0 if fwd else 1)
        self.pi.write(config.MOTOR_BR_1, 1 if fwd else 0)
        self.pi.write(config.MOTOR_BR_2, 0 if fwd else 1)
        # Only call hardware_PWM on GPIO 12 — GPIO 18 mirrors it automatically
        self.pi.hardware_PWM(12, config.MOTOR_PWM_FREQ, int(speed * 10000))

    def forward(self, speed=60):
        self._set_left(True,  speed)
        self._set_right(True, speed)

    def backward(self, speed=60):
        self._set_left(False,  speed)
        self._set_right(False, speed)

    def stop(self):
        self.pi.hardware_PWM(12, config.MOTOR_PWM_FREQ, 0)
        self.pi.hardware_PWM(13, config.MOTOR_PWM_FREQ, 0)

    def turn_left(self, speed=50):
        self._set_left(False, speed)
        self._set_right(True, speed)

    def turn_right(self, speed=50):
        self._set_left(True,  speed)
        self._set_right(False, speed)

    def smooth_left(self, speed=50):
        self._set_left(True,  speed * 0.5)
        self._set_right(True, speed)

    def smooth_right(self, speed=50):
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