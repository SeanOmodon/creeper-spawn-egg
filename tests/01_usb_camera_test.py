"""
Stage 1 — Camera + FPS test (USB camera via OpenCV)
Run: python3 tests/01_camera_test.py
Saves: /tmp/test_frame.jpg
"""

import time, sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
import config
import cv2

def main():
    print("[Stage 1] Opening USB camera...")
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  config.CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS,          config.CAMERA_FPS)

    if not cap.isOpened():
        print("[Stage 1] FAIL — could not open camera")
        sys.exit(1)

    print("[Stage 1] Warming up...")
    time.sleep(1)

    print(f"[Stage 1] Capturing {config.CAMERA_FPS * 3} frames to measure FPS...")
    frame_count   = 0
    target_frames = config.CAMERA_FPS * 3
    frame_sample  = None
    start = time.time()

    while frame_count < target_frames:
        ret, frame = cap.read()
        if not ret:
            print("[Stage 1] FAIL — could not read frame")
            cap.release()
            sys.exit(1)
        if frame_sample is None:
            frame_sample = frame
        frame_count += 1

    elapsed      = time.time() - start
    measured_fps = frame_count / elapsed
    cap.release()

    out_path = "/tmp/test_frame.jpg"
    cv2.imwrite(out_path, frame_sample)

    print(f"[Stage 1] Target FPS  : {config.CAMERA_FPS}")
    print(f"[Stage 1] Measured FPS: {measured_fps:.1f}")
    print(f"[Stage 1] Resolution  : {frame_sample.shape[1]}x{frame_sample.shape[0]}")
    print(f"[Stage 1] Sample frame: {out_path}")

    tolerance = 0.2
    if abs(measured_fps - config.CAMERA_FPS) / config.CAMERA_FPS <= tolerance:
        print("[Stage 1] PASS")
    else:
        print("[Stage 1] WARN — FPS outside 20% tolerance; lower CAMERA_FPS in config.py")

if __name__ == "__main__":
    main()