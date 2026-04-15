# Camera
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 15          # Pi Zero 2W sweet spot; drop to 10 if detection lags

# Motor GPIO (adjust to your wiring)
MOTOR_LEFT_FWD  = 17
MOTOR_LEFT_BWD  = 18
MOTOR_RIGHT_FWD = 22
MOTOR_RIGHT_BWD = 23
MOTOR_PWM_FREQ  = 100    # Hz

# Ultrasonic (HC-SR04)
TRIG_PIN = 24
ECHO_PIN = 25
OBSTACLE_DISTANCE_CM = 30   # stop threshold

# Detection
HOG_WIN_STRIDE   = (8, 8)
HOG_PADDING      = (4, 4)
HOG_SCALE        = 1.05
CONFIDENCE_FLOOR = 0.3      # filter weak detections