# config_odometry.py
# Configuration file with odometry parameters added

RUN_TRACK_NAME = "track_500"

# ---------------- Robot Physical Parameters ----------------
WHEELBASE_CM = 8.5  # Distance between left and right wheels in cm
WHEEL_DIAMETER_CM = 3.9  # Wheel diameter in cm
WHEEL_CIRCUMFERENCE_CM = WHEEL_DIAMETER_CM * 3.14159
ENCODER_COUNTS_PER_REVOLUTION = 900  # Encoder counts per wheel revolution
WHEEL_CM_PER_COUNT = WHEEL_CIRCUMFERENCE_CM / ENCODER_COUNTS_PER_REVOLUTION  # ~0.0136

# ---------------- Odometry & Navigation ----------------
NAV_POSITION_TOLERANCE = 3.0  # How close to target point in cm
NAV_ANGLE_TOLERANCE = 5.0  # How close to target heading in degrees
USE_IMU_FOR_HEADING = True  # Use IMU for heading (more accurate than encoders)

# ---------------- Gyro & Turn Control ----------------
GYRO_ANGLE_TOLERANCE = 2.0  # degrees, how close we must get to target
GYRO_TURN_KP = 500  # proportional gain for turning
GYRO_TURN_KD = 10  # derivative gain for turning
GYRO_MAX_SPEED = 2400  # max speed during turns
MOTOR_LEFT_TRIM = -110
MOTOR_RIGHT_TRIM = -10
TURN_ANGLE_SCALE = 2

GYRO_SETTLE_MS = 250  # must stay within tolerance this long
GYRO_MIN_TURN_SPEED = 40  # minimum turning command near the end

# ---------------- Behavior Overrides ----------------
IGNORE_OBSTACLES = False  # default: detect obstacles

# For heading hold while driving straight
GYRO_HOLD_KP = 12.0
GYRO_HOLD_KD = 0.0

# Obstacle detection thresholds and timing
OBSTACLE_THRESHOLD = 6
OBSTACLE_PAUSE = 0.5
BACKUP_TIME = 1.0

# ---------------- Turn configuration ----------------
TURN_SPEED = 2200
TURN_TICKS_PER_DEGREE = 5
TICKS_PER_DEGREE = 5.35
TURN_TICKS_90 = 445
ALLOW_BACKWARD = True

# ---------------- Drive configuration ----------------
DRIVE_SPEED = 400
TICKS_PER_CM = 19
CORRECTION_GAIN = 2
MAX_CORRECTION = 200
ALIGN_TOLERANCE = 2
SLOWDOWN_CM = 2
MIN_SPEED = 1000
SENSOR_POS = [-2, -1, 0, 1, 2]
MAX_SPEED = 2600 #MAX_SPEED = 2200
LINE_THRESHOLD = 700
TARGET_DISTANCE_CM = 50

# ---------------- Line Follower ----------------
LINE_MAX_SPEED = 2200 #LINE_MAX_SPEED = 2200
LINE_CALIBRATION_SPEED = 900
LINE_CALIBRATION_COUNT = 110
LINE_KP = 410
LINE_KD = 200
SCALE_FACTOR = 0.5
ENCODER_KP = 8.0

# Motor speeds for backup
BACKUP_SPEED = LINE_MAX_SPEED // 2

# ---------------- Proximity Sensor ----------------
PROX_FRONT_INDEX = 1  # Index of front proximity sensor pair
