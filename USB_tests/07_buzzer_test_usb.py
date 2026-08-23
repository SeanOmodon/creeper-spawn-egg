import time
import sys
import os
import threading
import random

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
import config

import onnxruntime as ort
import numpy as np
import cv2
import RPi.GPIO as GPIO

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
        self.pwm.ChangeFrequency(max(50, freq))
        self.pwm.ChangeDutyCycle(duty)

    def off(self):
        self.pwm.ChangeDutyCycle(0)

    def play_hiss_burst(self, start_freq, end_freq, duration, steps, led, brightness):
        step_time = duration / steps
        for i in range(steps):
            freq = int(start_freq + (end_freq - start_freq) * (i / steps))
            self.tone(freq)
            led.on(brightness)
            time.sleep(step_time)
        self.off()
        led.off()

    def play_creeper_hiss(self, led = None):
        num_bursts = 3
        speed      = 1
        brightness = 100
        gap        = 0.12

        stages = [
            (800, 200, 0.18, 30),
            (700, 150, 0.15, 25),
            (600, 100, 0.12, 20),
            (500,  80, 0.10, 18),
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

    def play_explosion(self, led = None):
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

def main():
    buzzer = BuzzerController()
    buzzer.off()
    buzzer.play_creeper_hiss()
    buzzer.off()
    buzzer.play_explosion()
    buzzer.off