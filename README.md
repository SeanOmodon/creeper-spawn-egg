# creeper-spawn-egg

Sequential computer vision pipeline tests for Raspberry Pi Zero 2W.

## Quick start

```bash
git clone https://github.com/YOUR_USERNAME/pi-cv-pipeline.git
cd pi-cv-pipeline
chmod +x setup.sh && ./setup.sh
```

## Test stages

| Script | Tests | Pass condition |
|---|---|---|
| `01_camera_test.py` | Camera + FPS | FPS within 20% of target |
| `02_person_detection.py` | OpenCV HOG detection | Bounding boxes in `/tmp/detections.jpg` |
| `03_yolo_detection.py` | OpenCV YOLO detection | Bounding boxes in `/tmp/detections.jpg` |
| `04_motor_control.py` | GPIO motor sequence | No GPIO errors |
| `05_ultrasonic_test.py` | HC-SR04 distance | Obstacle threshold triggers |
| `06_full_pipeline.py` | Full integration | Follow + obstacle stop |

Run each from the repo root:
```bash
python3 tests/01_camera_test.py
```

## Pin mapping (default — edit `config.py`)

| Signal | BCM pin |
|---|---|
| Motor L fwd | 17 |
| Motor L bwd | 18 |
| Motor R fwd | 22 |
| Motor R bwd | 23 |
| Ultrasonic TRIG | 24 |
| Ultrasonic ECHO | 25 |
