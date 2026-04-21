"""
Stage 3 — YOLOv8n person detection via ONNX (USB camera)
Run: python3 tests/03_yolo_detection.py
Saves: /tmp/yolo_detections.jpg
"""

import time, sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
import config
import onnxruntime as ort
import numpy as np
import cv2

def load_model():
    model_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "yolov8n.onnx")
    if not os.path.exists(model_path):
        print(f"[Stage 3] FAIL — yolov8n.onnx not found at {model_path}")
        print("[Stage 3] Export it on your laptop and scp it to the Pi — see README")
        sys.exit(1)
    session    = ort.InferenceSession(model_path)
    input_name = session.get_inputs()[0].name
    return session, input_name

def detect_people(session, input_name, bgr):
    img       = cv2.resize(bgr, (320, 320))
    img_input = img.transpose(2, 0, 1)[np.newaxis].astype(np.float32) / 255.0
    outputs   = session.run(None, {input_name: img_input})[0][0]  # shape: (84, 8400)

    boxes = []
    for i in range(outputs.shape[1]):
        scores   = outputs[4:, i]
        class_id = int(np.argmax(scores))
        conf     = float(scores[class_id])
        if class_id != 0 or conf < config.CONFIDENCE_FLOOR:
            continue
        cx, cy, w, h = outputs[:4, i]
        x1 = int((cx - w / 2) * (config.CAMERA_WIDTH  / 320))
        y1 = int((cy - h / 2) * (config.CAMERA_HEIGHT / 320))
        x2 = int((cx + w / 2) * (config.CAMERA_WIDTH  / 320))
        y2 = int((cy + h / 2) * (config.CAMERA_HEIGHT / 320))
        boxes.append((x1, y1, x2, y2, conf))
    return boxes

def main():
    print("[Stage 3] Loading YOLOv8n ONNX model...")
    session, input_name = load_model()

    print("[Stage 3] Opening USB camera...")
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  config.CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS,          config.CAMERA_FPS)

    if not cap.isOpened():
        print("[Stage 3] FAIL — could not open camera")
        sys.exit(1)

    time.sleep(1)

    print("[Stage 3] Running detection for 30 seconds — walk in front of the camera!")
    deadline         = time.time() + 30
    total_detections = 0
    frames_processed = 0
    last_save        = None

    while time.time() < deadline:
        ret, frame = cap.read()
        if not ret:
            print("[Stage 3] WARN — failed to read frame, skipping")
            continue

        boxes = detect_people(session, input_name, frame)

        for (x1, y1, x2, y2, conf) in boxes:
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"person {conf:.2f}", (x1, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            total_detections += 1
            last_save = frame.copy()

        frames_processed += 1
        if frames_processed % 10 == 0:
            remaining = int(deadline - time.time())
            print(f"[Stage 3] {remaining}s left | detections this frame: {len(boxes)}")

    cap.release()

    out_path = "/tmp/yolo_detections.jpg"
    cv2.imwrite(out_path, last_save if last_save is not None else frame)

    print(f"[Stage 3] Frames processed : {frames_processed}")
    print(f"[Stage 3] Total detections  : {total_detections}")
    print(f"[Stage 3] Saved             : {out_path}")
    print("[Stage 3] PASS")

if __name__ == "__main__":
    main()
