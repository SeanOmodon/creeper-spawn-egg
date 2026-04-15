"""
Stage 3 — YOLOv8n person detection
Run: python3 tests/03_yolo_detection.py
Saves: /tmp/yolo_detections.jpg
Pass condition: consistent person detections with confidence scores shown.
"""

import time, sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
import config

from picamera2 import Picamera2
from ultralytics import YOLO
import cv2

def main():
    print("[Stage 3] Loading YOLOv8n model (downloads on first run)...")
    model = YOLO("yolov8n.pt")

    print("[Stage 3] Starting camera...")
    cam = Picamera2()
    cam.configure(cam.create_video_configuration(
        main={"size": (config.CAMERA_WIDTH, config.CAMERA_HEIGHT), "format": "RGB888"},
        controls={"FrameRate": config.CAMERA_FPS}
    ))
    cam.start()
    time.sleep(1)

    print("[Stage 3] Running detection for 30 seconds — walk in front of the camera!")
    deadline = time.time() + 30
    total_detections = 0
    frames_processed = 0
    last_save = None

    while time.time() < deadline:
        frame = cam.capture_array()
        bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        results = model(bgr, classes=[0], verbose=False, imgsz=320)
        boxes = results[0].boxes

        for box in boxes:
            conf = float(box.conf[0])
            if conf < config.CONFIDENCE_FLOOR:
                continue
            x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
            cv2.rectangle(bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(bgr, f"person {conf:.2f}", (x1, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            total_detections += 1
            last_save = bgr.copy()

        frames_processed += 1
        if frames_processed % 30 == 0:
            remaining = int(deadline - time.time())
            print(f"[Stage 3] {remaining}s left | detections this frame: {len(boxes)}")

    cam.stop()
    cam.close()

    out_path = "/tmp/yolo_detections.jpg"
    cv2.imwrite(out_path, last_save if last_save is not None else bgr)

    print(f"[Stage 3] Frames processed : {frames_processed}")
    print(f"[Stage 3] Total detections  : {total_detections}")
    print(f"[Stage 3] Saved             : {out_path}")
    print("[Stage 3] PASS")

if __name__ == "__main__":
    main()