import time
import sys
import os
# import threading
# import random

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
import config

# import onnxruntime as ort
# import numpy as np
# import cv2
import RPi.GPIO as GPIO

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

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
        self.pwm.ChangeFrequency(freq)
        self.pwm.ChangeDutyCycle(duty)

    def off(self):
        self.pwm.ChangeDutyCycle(0)

    def play_hiss_burst(self, start_freq, end_freq, duration, steps, led, brightness, duty=40):
        step_time = duration / steps
        for i in range(steps):
            freq = int(start_freq + (end_freq - start_freq) * (i / steps))
            self.tone(freq, duty)
            # led.on(brightness)
            time.sleep(step_time)
        self.off()
        # led.off()

    def play_creeper_hiss(self, led = None):
        brightness = 100

        stages = [
		(50, 200, 0.2, 15, 0.3),
		(200, 50, 0.5, 20, 0),
		(50, 175, 0.5, 20, 0),
        (175, 100, 0.1, 5, 0)
		]

        for i in range(len(stages)):
            if (i == len(stages) - 1):
                for j in range(4):
                    for k in range(2):
                        start_freq, end_freq, duration, steps, wait = stages[i]
                        self.play_hiss_burst(
                            start_freq, end_freq,
                            duration, steps,
                            led, brightness
                        )
                        end_freq, start_freq, duration, steps, wait = stages[i]
                        self.play_hiss_burst(
                            start_freq, end_freq,
                            duration, steps,
                            led, brightness
                        )
                    time.sleep(0.3) 
            else: 
                start_freq, end_freq, duration, steps, wait = stages[i]
                self.play_hiss_burst(
                    start_freq, end_freq,
                    duration, steps,
                    led, brightness
                )
                time.sleep(wait)

    def play_explosion(self, led = None):
        explosion_seq = [
            (3000,0.06),(3500,0.03),(3000,0.02),(3500,0.03),(2000,0.06),(1000,0.06)
        ]
        for i, (freq, duration) in enumerate(explosion_seq):
            self.tone(freq)
            # led.on(100) if i % 2 == 0 else led.off()
            time.sleep(duration)
        self.off()
        # led.off()

    def cleanup(self):
        self.off()
        self.pwm.stop()

def main():
    buzzer = BuzzerController()
    print("Buzzer controller initialized.")
    buzzer.off()
    print("Buzzer playing creeper hiss.")
    buzzer.play_creeper_hiss()
    buzzer.off()
    time.sleep(1)
    print("Buzzer playing explosion.")
    buzzer.play_explosion()
    buzzer.off

if __name__ == "__main__":
    main()
