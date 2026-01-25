"""
Conduit IMU sensor implementation using Zenoh.

Receives IMU data from iPhone running the Conduit app via Zenoh protocol.
No ROS2 dependency required - uses direct Zenoh subscription with CDR parsing.

Conduit app: https://apps.apple.com/us/app/conduit-powered-by-ros/id6757171237
Documentation: https://github.com/youtalk/conduit-support
"""
import os
import time
import threading
from collections import deque
from dataclasses import dataclass
from typing import Optional

from cereal import log
from openpilot.common.swaglog import cloudlog

from openpilot.system.sensord.sensors.cdr_parser import parse_imu_cdr

# Zenoh import - optional dependency
try:
  import zenoh
  ZENOH_AVAILABLE = True
except ImportError:
  ZENOH_AVAILABLE = False
  cloudlog.warning("zenoh-python not installed. Conduit sensor unavailable.")


# Environment variables for configuration
CONDUIT_ROUTER = os.environ.get("CONDUIT_ROUTER", "tcp/192.168.1.100:7447")
CONDUIT_DOMAIN_ID = int(os.environ.get("CONDUIT_DOMAIN_ID", "0"))
CONDUIT_IMU_TOPIC = os.environ.get("CONDUIT_IMU_TOPIC", "conduit/imu")

# Buffer size for incoming IMU messages
IMU_BUFFER_SIZE = 10


@dataclass
class ImuData:
  """Parsed IMU data with timestamp."""
  timestamp_ns: int
  accel_x: float  # m/s^2
  accel_y: float  # m/s^2
  accel_z: float  # m/s^2
  gyro_x: float   # rad/s
  gyro_y: float   # rad/s
  gyro_z: float   # rad/s


class ConduitIMU:
  """
  Conduit IMU sensor that receives data from iPhone via Zenoh.

  The Conduit app publishes sensor_msgs/Imu at 100Hz using Zenoh.
  This class subscribes to the IMU topic and provides accelerometer
  and gyroscope events compatible with openpilot's messaging system.
  """

  def __init__(self, router: str = CONDUIT_ROUTER):
    if not ZENOH_AVAILABLE:
      raise RuntimeError("zenoh-python is required for Conduit sensor. Install with: pip install eclipse-zenoh")

    self.router = router
    self.session: Optional[zenoh.Session] = None
    self.subscriber = None
    self._running = False
    self._connected = False
    self._data_buffer: deque[ImuData] = deque(maxlen=IMU_BUFFER_SIZE)
    self._lock = threading.Lock()
    self._last_recv_time = 0.0
    self.start_ts = 0.0
    self.source = log.SensorEventData.SensorSource.iOS

  def _on_imu_sample(self, sample: 'zenoh.Sample') -> None:
    """Callback for incoming IMU messages."""
    try:
      # Parse CDR-encoded sensor_msgs/Imu
      imu_msg = parse_imu_cdr(bytes(sample.payload))

      # Convert to internal format
      # Note: Conduit/iOS uses ENU frame, openpilot expects specific orientation
      # The axis mapping may need adjustment based on phone mounting
      imu_data = ImuData(
        timestamp_ns=imu_msg.header.stamp.to_ns(),
        accel_x=imu_msg.linear_acceleration.x,
        accel_y=imu_msg.linear_acceleration.y,
        accel_z=imu_msg.linear_acceleration.z,
        gyro_x=imu_msg.angular_velocity.x,
        gyro_y=imu_msg.angular_velocity.y,
        gyro_z=imu_msg.angular_velocity.z,
      )

      with self._lock:
        self._data_buffer.append(imu_data)
        self._last_recv_time = time.monotonic()

    except Exception as e:
      cloudlog.exception(f"Error parsing Conduit IMU message: {e}")

  def init(self) -> None:
    """Initialize Zenoh connection and subscribe to IMU topic."""
    cloudlog.info(f"Connecting to Conduit via Zenoh at {self.router}")

    try:
      # Configure Zenoh session
      config = zenoh.Config()

      # Set up connection to Conduit router
      # Format: tcp/IP:PORT or udp/IP:PORT
      config.insert_json5("connect/endpoints", f'["{self.router}"]')

      # Open session
      self.session = zenoh.open(config)
      cloudlog.info("Zenoh session opened")

      # Subscribe with wildcard pattern to match Conduit IMU topic
      # rmw_zenoh key format: <domain_id>/<topic_name>/<type_name>
      primary_key = f"**/{CONDUIT_IMU_TOPIC}"
      self.subscriber = self.session.declare_subscriber(
        primary_key,
        self._on_imu_sample
      )
      cloudlog.info(f"Subscribed to Conduit IMU topic: {primary_key}")

      self._running = True
      self._connected = True
      self.start_ts = time.monotonic()

    except Exception as e:
      cloudlog.exception(f"Failed to connect to Conduit: {e}")
      raise

  def shutdown(self) -> None:
    """Close Zenoh connection."""
    self._running = False
    self._connected = False

    if self.subscriber is not None:
      try:
        self.subscriber.undeclare()
      except Exception:
        pass
      self.subscriber = None

    if self.session is not None:
      try:
        self.session.close()
      except Exception:
        pass
      self.session = None

    cloudlog.info("Conduit IMU sensor shutdown")

  def is_connected(self) -> bool:
    """Check if receiving data from Conduit."""
    if not self._connected:
      return False
    # Consider disconnected if no data for 1 second
    return (time.monotonic() - self._last_recv_time) < 1.0

  def is_data_valid(self) -> bool:
    """Check if sensor data is valid (warmup period passed)."""
    if self.start_ts == 0:
      self.start_ts = time.monotonic()
    return (time.monotonic() - self.start_ts) > 0.5

  def get_latest_imu(self) -> Optional[ImuData]:
    """Get the latest IMU data from buffer."""
    with self._lock:
      if self._data_buffer:
        return self._data_buffer[-1]
    return None

  def get_accel_event(self, ts: Optional[int] = None) -> log.SensorEventData:
    """
    Get accelerometer event in openpilot format.

    Args:
      ts: Optional timestamp in nanoseconds. If None, uses latest IMU timestamp.

    Returns:
      SensorEventData with acceleration data.

    Raises:
      RuntimeError: If no IMU data available.
    """
    imu_data = self.get_latest_imu()
    if imu_data is None:
      raise RuntimeError("No IMU data available from Conduit")

    if ts is None:
      ts = imu_data.timestamp_ns

    event = log.SensorEventData.new_message()
    event.timestamp = ts
    event.version = 1
    event.sensor = 1  # SENSOR_ACCELEROMETER
    event.type = 1    # SENSOR_TYPE_ACCELEROMETER
    event.source = self.source

    # Axis mapping for iPhone mounted in specific orientation
    # This may need adjustment based on actual phone mounting position
    # Assuming phone is mounted with screen facing up, top pointing forward
    # iPhone coordinate system: X right, Y up, Z out of screen
    # Vehicle coordinate system: X forward, Y left, Z up
    a = event.init('acceleration')
    a.v = [
      float(imu_data.accel_y),   # Forward = iPhone Y
      float(-imu_data.accel_x),  # Left = -iPhone X
      float(imu_data.accel_z),   # Up = iPhone Z
    ]
    a.status = 1

    return event

  def get_gyro_event(self, ts: Optional[int] = None) -> log.SensorEventData:
    """
    Get gyroscope event in openpilot format.

    Args:
      ts: Optional timestamp in nanoseconds. If None, uses latest IMU timestamp.

    Returns:
      SensorEventData with gyroscope data.

    Raises:
      RuntimeError: If no IMU data available.
    """
    imu_data = self.get_latest_imu()
    if imu_data is None:
      raise RuntimeError("No IMU data available from Conduit")

    if ts is None:
      ts = imu_data.timestamp_ns

    event = log.SensorEventData.new_message()
    event.timestamp = ts
    event.version = 2
    event.sensor = 5  # SENSOR_GYRO_UNCALIBRATED
    event.type = 16   # SENSOR_TYPE_GYROSCOPE_UNCALIBRATED
    event.source = self.source

    # Axis mapping (same as accelerometer)
    g = event.init('gyroUncalibrated')
    g.v = [
      float(imu_data.gyro_y),   # Forward rotation
      float(-imu_data.gyro_x),  # Left rotation
      float(imu_data.gyro_z),   # Up rotation (yaw)
    ]
    g.status = 1

    return event


if __name__ == "__main__":
  # Test the Conduit IMU sensor
  import argparse

  parser = argparse.ArgumentParser(description="Test Conduit IMU sensor")
  parser.add_argument("--router", default=CONDUIT_ROUTER, help="Zenoh router address")
  args = parser.parse_args()

  if not ZENOH_AVAILABLE:
    print("Error: zenoh-python not installed. Install with: pip install eclipse-zenoh")
    exit(1)

  print(f"Connecting to Conduit at {args.router}...")

  sensor = ConduitIMU(router=args.router)
  try:
    sensor.init()
    print("Connected! Waiting for IMU data...")

    for _ in range(100):
      time.sleep(0.1)
      if sensor.is_connected():
        imu = sensor.get_latest_imu()
        if imu:
          print(f"Accel: [{imu.accel_x:.3f}, {imu.accel_y:.3f}, {imu.accel_z:.3f}] Gyro: [{imu.gyro_x:.3f}, {imu.gyro_y:.3f}, {imu.gyro_z:.3f}]")
      else:
        print("Waiting for Conduit connection...")

  except KeyboardInterrupt:
    print("\nInterrupted")
  finally:
    sensor.shutdown()
