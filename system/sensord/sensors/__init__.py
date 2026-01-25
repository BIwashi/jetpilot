# Sensor implementations for sensord
#
# Hardware sensors:
#   - LSM6DS3_Accel: Accelerometer (I2C)
#   - LSM6DS3_Gyro: Gyroscope (I2C)
#   - LSM6DS3_Temp: Temperature sensor (I2C)
#   - MMC5603NJ_Magn: Magnetometer (I2C)
#
# Network sensors:
#   - ConduitIMU: iPhone IMU via Zenoh (Conduit app)
#
# ============================================================================
# CONDUIT IMU CONFIGURATION
# ============================================================================
#
# Environment variables:
#   CONDUIT_IMU=1                   Enable Conduit IMU instead of hardware sensors
#   CONDUIT_ROUTER                  Zenoh router address (default: tcp/192.168.1.100:7447)
#   CONDUIT_DOMAIN_ID               ROS2 domain ID (default: 0)
#   CONDUIT_IMU_TOPIC               IMU topic name (default: conduit/imu)
#   CONDUIT_MOUNT_ORIENTATION       iPhone mounting orientation (default: up,forward)
#   CONDUIT_ENABLE_CALIBRATION      Enable calibration (default: 1)
#   CONDUIT_STATIC_CALIBRATION      Start static calibration on launch (default: 0)
#
# ----------------------------------------------------------------------------
# MOUNT ORIENTATION
# ----------------------------------------------------------------------------
# Format: "SCREEN_DIRECTION,NOTCH_DIRECTION"
#   SCREEN_DIRECTION: up, down, forward, backward (which way the screen faces)
#   NOTCH_DIRECTION: forward, backward, left, right, up, down (which way the notch points)
#
# Common configurations:
#   "up,forward"      - Screen facing sky, notch pointing forward (dashboard flat mount)
#   "up,backward"     - Screen facing sky, notch pointing backward
#   "forward,up"      - Screen facing forward, notch pointing up (windshield mount)
#   "backward,left"   - Screen facing driver, notch left, charging port right (MagSafe vertical)
#   "backward,right"  - Screen facing driver, notch right, charging port left (MagSafe vertical)
#   "down,forward"    - Screen facing ground, notch pointing forward (under-dash mount)
#
# ----------------------------------------------------------------------------
# CALIBRATION
# ----------------------------------------------------------------------------
# Two calibration methods are available:
#
# 1. Static Calibration (recommended for initial setup):
#    - Start with: CONDUIT_STATIC_CALIBRATION=1
#    - Keep vehicle stationary on level ground for ~2 seconds
#    - Calculates gyro bias and roll/pitch offset from gravity
#    - Saved to /data/params/d/ConduitIMUCalibration
#
# 2. Online Calibration (automatic during driving):
#    - Learns from straight-line driving segments
#    - Refines gyro bias over time
#    - Requires vehicle speed > 5 m/s
#
# To view current calibration:
#    python -m openpilot.system.sensord.sensors.conduit_calibration --show
#
# To run static calibration test:
#    python -m openpilot.system.sensord.sensors.conduit_calibration --static
