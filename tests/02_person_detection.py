"""
Stage 2 — OpenCV HOG person detection
Run: python3 tests/02_person_detection.py
Saves: /tmp/detections.jpg
Pass condition: no crashes; detection loop runs for 30s; frame with boxes saved.
"""

import time, sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
import config

from picamera2 import Picamera2
import cv2
import numpy as np

def build_hog():
    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
    return hog

def detect_people(hog, frame_bgr):
    """Returns list of (x, y, w, h) bounding boxes."""
    # Downscale for speed on Pi Zero 2W
    small = cv2.resize(frame_bgr, (320, 240))
    rects, weights = hog.detectMultiScale(
        small,
        winStride=config.HOG_WIN_STRIDE,
        padding=config.HOG_PADDING,
        scale=config.HOG_SCALE
    )
    if len(rects) == 0:
        return []
    # Filter low-confidence and scale back to full res
    scale_x = frame_bgr.shape[1] / 320
    scale_y = frame_bgr.shape[0] / 240
    results = []
    for (x, y, w, h), weight in zip(rects, weights):
        if float(weight) < config.CONFIDENCE_FLOOR:
            continue
        results.append((
            int(x * scale_x), int(y * scale_y),
            int(w * scale_x), int(h * scale_y)
        ))
    return results

def main():
    print("[Stage 2] Building HOG detector...")
    hog = build_hog()

    print("[Stage 2] Starting camera...")
    cam = Picamera2()
    cam.configure(cam.create_video_configuration(
        main={"size": (config.CAMERA_WIDTH, config.CAMERA_HEIGHT), "format": "RGB888"},
        controls={"FrameRate": config.CAMERA_FPS}
    ))
    cam.start()
    time.sleep(1)

    print("[Stage 2] Running detection for 30 seconds — walk in front of the camera!")
    deadline = time.time() + 30
    last_save = None
    total_detections = 0
    frames_processed = 0

    while time.time() < deadline:
        frame = cam.capture_array()
        bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        boxes = detect_people(hog, bgr)
        total_detections += len(boxes)
        frames_processed += 1

        for (x, y, w, h) in boxes:
            cv2.rectangle(bgr, (x, y), (x + w, y + h), (0, 255, 0), 2)

        if boxes:
            last_save = bgr.copy()

        remaining = int(deadline - time.time())
        if frames_processed % 30 == 0:
            print(f"[Stage 2] {remaining}s left | detections this frame: {len(boxes)}")

    cam.stop()
    cam.close()

    out_path = "/tmp/detections.jpg"
    save_frame = last_save if last_save is not None else bgr
    cv2.imwrite(out_path, save_frame)

    print(f"[Stage 2] Frames processed : {frames_processed}")
    print(f"[Stage 2] Total detections  : {total_detections}")
    print(f"[Stage 2] Last detection img: {out_path}")
    print("[Stage 2] PASS — review /tmp/detections.jpg to confirm bounding boxes")

if __name__ == "__main__":
    main()