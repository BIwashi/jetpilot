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
# Environment variables for Conduit IMU:
#   CONDUIT_IMU=1           Enable Conduit IMU instead of hardware sensors
#   CONDUIT_ROUTER          Zenoh router address (default: tcp/192.168.1.100:7447)
#   CONDUIT_DOMAIN_ID       ROS2 domain ID (default: 0)
#   CONDUIT_IMU_TOPIC       IMU topic name (default: conduit/imu)
