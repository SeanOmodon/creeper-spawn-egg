CAMERA_WIDTH    = 640
CAMERA_HEIGHT   = 480
CAMERA_FPS      = 30    # Pi 4 handles 30fps fine
DEAD_ZONE        = 60       # px offset from center where we still go straight
RESTART_DISTANCE_CM = 3   # cm — touching the sensor counts as restart

# Motor GPIO 
MOTOR_FL_1      = 2     # Front Left IN4
MOTOR_FL_2      = 3     # Front Left IN3
MOTOR_FL_EnB    = 13    # Front Left EnB

MOTOR_FR_1      = 0     # Front Right IN2
MOTOR_FR_2      = 1     # Front Right IN1
MOTOR_FR_EnA    = 12    # Front Right EnA

MOTOR_BR_1      = 7     # Back Right IN3
MOTOR_BR_2      = 6     # Back Right IN4
MOTOR_BR_EnB    = 18    # Back Right EnB

MOTOR_BL_1      = 5     # Back Left IN2
MOTOR_BL_2      = 4     # Back Left IN1
MOTOR_BL_EnA    = 19    # Back Left EnA

MOTOR_PWM_FREQ  = 100   # Hz

# Ultrasonic (HC-SR04)
TRIG_PIN = 10
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
CONFIDENCE_FLOOR = 0.35      # filter weak detections
