"""
Stage 2 — OpenCV HOG person detection (USB camera)
Run: python3 tests/02_person_detection.py
Saves: /tmp/detections.jpg
Pass condition: no crashes; detection loop runs for 30s; frame with boxes saved.
"""

import time, sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
import config
import cv2
import numpy as np

def build_hog():
    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
    return hog

def detect_people(hog, frame_bgr):
    """Returns list of (x, y, w, h) bounding boxes."""
    small = cv2.resize(frame_bgr, (320, 240))
    rects, weights = hog.detectMultiScale(
        small,
        winStride=config.HOG_WIN_STRIDE,
        padding=config.HOG_PADDING,
        scale=config.HOG_SCALE
    )
    if len(rects) == 0:
        return []
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

    print("[Stage 2] Opening USB camera...")
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  config.CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS,          config.CAMERA_FPS)

    if not cap.isOpened():
        print("[Stage 2] FAIL — could not open camera")
        sys.exit(1)

    time.sleep(1)

    print("[Stage 2] Running detection for 30 seconds — walk in front of the camera!")
    deadline = time.time() + 30
    last_save = None
    total_detections = 0
    frames_processed = 0

    while time.time() < deadline:
        ret, frame = cap.read()
        if not ret:
            print("[Stage 2] WARN — failed to read frame, skipping")
            continue

        boxes = detect_people(hog, frame)
        total_detections += len(boxes)
        frames_processed += 1

        for (x, y, w, h) in boxes:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        if boxes:
            last_save = frame.copy()

        remaining = int(deadline - time.time())
        if frames_processed % 30 == 0:
            print(f"[Stage 2] {remaining}s left | detections this frame: {len(boxes)}")

    cap.release()

    out_path = "/tmp/detections.jpg"
    save_frame = last_save if last_save is not None else frame
    cv2.imwrite(out_path, save_frame)

    print(f"[Stage 2] Frames processed : {frames_processed}")
    print(f"[Stage 2] Total detections  : {total_detections}")
    print(f"[Stage 2] Last detection img: {out_path}")
    print("[Stage 2] PASS — review /tmp/detections.jpg to confirm bounding boxes")

if __name__ == "__main__":
    main()
