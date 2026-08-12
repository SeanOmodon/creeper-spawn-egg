CAMERA_WIDTH    = 640
CAMERA_HEIGHT   = 480
CAMERA_FPS      = 24    # Pi 4 handles 30fps fine
DEAD_ZONE        = 120       # px offset from center where we still go straight
RESTART_DISTANCE_CM = 3   # cm — touching the sensor counts as restart
IDLE_OBSTACLE_CM = 20   # closer than this = obstacle in idle mode
IDLE_SPEED = 25   # was 40 — lower peak current in idle mode, but still enough to move
NO_MOTION_CHANCE = 0.35   # chance of not moving at all in idle mode

# Motor GPIO 
MOTOR_F_4      = 6     # Front Left IN4
MOTOR_F_3      = 7     # Front Left IN3
MOTOR_F_EnB    = 26    # Front Left EnB

MOTOR_F_2      = 0     # Front Right IN2
MOTOR_F_1      = 1     # Front Right IN1
MOTOR_F_EnA    = 12    # Front Right EnA

MOTOR_B_4      = 2     # Back Left IN4
MOTOR_B_3      = 3     # Back Left IN3
MOTOR_B_EnB    = 13    # Back Left EnB

MOTOR_B_2      = 5     # Back Right IN2
MOTOR_B_1      = 4     # Back Right IN1
MOTOR_B_EnA    = 19    # Back Right EnA

MOTOR_PWM_FREQ  = 100   # Hz

# Ultrasonic (HC-SR04)
TRIG_PIN = 10
ECHO_PIN = 8
ECHO_PIN_FRONT = 8
ECHO_PIN_BACK = 9
OBSTACLE_DISTANCE_CM = 30   # stop threshold

# LED RGB Strip
LED_R = 15
# LED_G = 14
LED_WHITE = 14
LED_B = 16
LED_PWM_FREQ = 200

# Creeper Eyes

RED_LED = 17

# Proximity Test

BUZZER_PIN = 20
ALERT_DISTANCE_CM   = 67    # Trigger distance
ALERT_MIN_INTERVAL   = 0.1   # Fastest beep interval (sec) at very close range
ALERT_MAX_INTERVAL  = 0.8   # slowest beep interval (sec) at trigger distance

# Detection
HOG_WIN_STRIDE   = (8, 8)
HOG_PADDING      = (4, 4)
HOG_SCALE        = 1.05
CONFIDENCE_FLOOR = 0.5      # filter weak detections
