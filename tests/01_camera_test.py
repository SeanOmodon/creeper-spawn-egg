"""
Stage 1 — Camera + FPS test
Run: python3 tests/01_camera_test.py
Saves: /tmp/test_frame.jpg
Pass condition: measured FPS within 20% of target, frame saved cleanly.
"""

import time
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
import config

from picamera2 import Picamera2
import cv2

def main():
    print("[Stage 1] Initialising camera...")
    cam = Picamera2()
    cam_cfg = cam.create_video_configuration(
        main={"size": (config.CAMERA_WIDTH, config.CAMERA_HEIGHT), "format": "RGB888"},
        controls={"FrameRate": config.CAMERA_FPS}
    )
    cam.configure(cam_cfg)
    cam.start()
    time.sleep(1)  # warm-up

    print(f"[Stage 1] Capturing {config.CAMERA_FPS * 3} frames to measure FPS...")
    frame_count = 0
    target_frames = config.CAMERA_FPS * 3
    start = time.time()

    frame_sample = None
    while frame_count < target_frames:
        frame = cam.capture_array()
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if frame_sample is None:
            frame_sample = frame_bgr
        frame_count += 1

    elapsed = time.time() - start
    measured_fps = frame_count / elapsed

    cam.stop()
    cam.close()

    # Save sample frame
    out_path = "/tmp/test_frame.jpg"
    cv2.imwrite(out_path, frame_sample)

    print(f"[Stage 1] Target FPS : {config.CAMERA_FPS}")
    print(f"[Stage 1] Measured FPS: {measured_fps:.1f}")
    print(f"[Stage 1] Sample frame : {out_path}")

    tolerance = 0.2
    if abs(measured_fps - config.CAMERA_FPS) / config.CAMERA_FPS <= tolerance:
        print("[Stage 1] PASS")
    else:
        print("[Stage 1] WARN — FPS outside 20% tolerance; consider lowering CAMERA_FPS in config.py")

if __name__ == "__main__":
    main()