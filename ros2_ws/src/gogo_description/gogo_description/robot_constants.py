# gogo_config/robot_constants.py

from math import pi

# Physical constants
WHEEL_RADIUS = 0.038          # meters
WHEEL_SEPARATION = 0.278      # meters
ENCODER_CPR = 4741.34

# Motor limits
MIN_TPS = 0
MAX_TPS = 2000

# LIDAR mounting pose relative to base_link
LIDAR_X = 0.0
LIDAR_Y = 0.0
LIDAR_Z = 0.16
LIDAR_ROLL = 0.0
LIDAR_PITCH = 0.0
LIDAR_YAW = 0.0

# Firmware
BAUDRATE = 38400
DEFAULT_PORT = "/dev/ttyACM0"
