"""
IMU Calibration for Conduit iPhone sensor.

Provides two calibration methods:
1. Static calibration: Calibrate while vehicle is stationary on level ground
2. Online calibration: Learn corrections from driving data over time

Calibration data is stored in /data/params/d/ConduitIMUCalibration
"""
import json
import math
import os
import time
from dataclasses import dataclass, field, asdict
from typing import Optional
from collections import deque

from openpilot.common.swaglog import cloudlog


# Calibration file path
CALIBRATION_PATH = "/data/params/d/ConduitIMUCalibration"

# Gravity constant (m/s^2)
GRAVITY = 9.81

# Static calibration settings
STATIC_SAMPLE_COUNT = 200  # Number of samples to collect (at 100Hz = 2 seconds)
STATIC_MOTION_THRESHOLD = 0.1  # Maximum allowed motion (rad/s) during static cal

# Online calibration settings
ONLINE_LEARNING_RATE = 0.001  # How fast to adapt (smaller = more stable)
ONLINE_MIN_SPEED = 5.0  # Minimum speed (m/s) for online calibration
ONLINE_MAX_GYRO = 0.1  # Maximum gyro magnitude for "straight" detection (rad/s)


@dataclass
class CalibrationData:
  """IMU calibration parameters."""
  # Accelerometer bias (m/s^2)
  accel_bias: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])

  # Gyroscope bias (rad/s)
  gyro_bias: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])

  # Roll and pitch offset (radians) - rotation from sensor frame to vehicle frame
  roll_offset: float = 0.0
  pitch_offset: float = 0.0

  # Calibration status
  static_calibrated: bool = False
  online_calibrated: bool = False
  online_sample_count: int = 0

  # Timestamp of last calibration
  last_calibration_time: float = 0.0

  def to_json(self) -> str:
    return json.dumps(asdict(self))

  @classmethod
  def from_json(cls, json_str: str) -> "CalibrationData":
    try:
      data = json.loads(json_str)
      return cls(**data)
    except (json.JSONDecodeError, TypeError) as e:
      cloudlog.warning(f"Failed to parse calibration data: {e}")
      return cls()


def load_calibration() -> CalibrationData:
  """Load calibration data from file."""
  if os.path.exists(CALIBRATION_PATH):
    try:
      with open(CALIBRATION_PATH) as f:
        return CalibrationData.from_json(f.read())
    except Exception as e:
      cloudlog.warning(f"Failed to load calibration: {e}")
  return CalibrationData()


def save_calibration(cal: CalibrationData) -> None:
  """Save calibration data to file."""
  try:
    os.makedirs(os.path.dirname(CALIBRATION_PATH), exist_ok=True)
    with open(CALIBRATION_PATH, "w") as f:
      f.write(cal.to_json())
    cloudlog.info("Calibration saved")
  except Exception as e:
    cloudlog.error(f"Failed to save calibration: {e}")


class StaticCalibrator:
  """
  Static IMU calibration.

  Calibrates the IMU while the vehicle is stationary on level ground.
  Calculates:
  - Gyroscope bias from average gyro readings (should be zero when stationary)
  - Roll/pitch offset from gravity vector direction
  """

  def __init__(self):
    self.accel_samples: list[list[float]] = []
    self.gyro_samples: list[list[float]] = []
    self.calibrating = False
    self.calibration_complete = False
    self.result: Optional[CalibrationData] = None

  def start(self) -> None:
    """Start static calibration. Vehicle must be stationary on level ground."""
    self.accel_samples = []
    self.gyro_samples = []
    self.calibrating = True
    self.calibration_complete = False
    self.result = None
    cloudlog.info("Static calibration started. Keep vehicle stationary on level ground.")

  def add_sample(self, accel: list[float], gyro: list[float]) -> bool:
    """
    Add a sample during calibration.

    Args:
      accel: Accelerometer reading [x, y, z] in m/s^2
      gyro: Gyroscope reading [x, y, z] in rad/s

    Returns:
      True if calibration is complete
    """
    if not self.calibrating:
      return False

    # Check for motion
    gyro_magnitude = math.sqrt(sum(g**2 for g in gyro))
    if gyro_magnitude > STATIC_MOTION_THRESHOLD:
      cloudlog.warning(f"Motion detected during calibration (gyro={gyro_magnitude:.3f}). Restarting...")
      self.accel_samples = []
      self.gyro_samples = []
      return False

    self.accel_samples.append(accel)
    self.gyro_samples.append(gyro)

    if len(self.accel_samples) >= STATIC_SAMPLE_COUNT:
      self._compute_calibration()
      return True

    return False

  def _compute_calibration(self) -> None:
    """Compute calibration from collected samples."""
    # Average accelerometer and gyroscope readings
    accel_avg = [
      sum(s[i] for s in self.accel_samples) / len(self.accel_samples)
      for i in range(3)
    ]
    gyro_avg = [
      sum(s[i] for s in self.gyro_samples) / len(self.gyro_samples)
      for i in range(3)
    ]

    # Gyro bias is simply the average (should be zero when stationary)
    gyro_bias = gyro_avg

    # Calculate roll and pitch from gravity vector
    # When level: accel should be [0, 0, GRAVITY]
    # Roll: rotation around X axis (forward)
    # Pitch: rotation around Y axis (left)
    ax, ay, az = accel_avg
    accel_magnitude = math.sqrt(ax**2 + ay**2 + az**2)

    # Normalize to gravity
    if accel_magnitude > 0:
      ax /= accel_magnitude / GRAVITY
      ay /= accel_magnitude / GRAVITY
      az /= accel_magnitude / GRAVITY

    # Roll = atan2(ay, az) - expected roll is 0 when level
    # Pitch = atan2(-ax, sqrt(ay^2 + az^2)) - expected pitch is 0 when level
    roll_offset = math.atan2(ay, az)
    pitch_offset = math.atan2(-ax, math.sqrt(ay**2 + az**2))

    # Create calibration result
    self.result = CalibrationData(
      accel_bias=[0.0, 0.0, 0.0],  # We use roll/pitch offset instead of accel bias
      gyro_bias=gyro_bias,
      roll_offset=roll_offset,
      pitch_offset=pitch_offset,
      static_calibrated=True,
      online_calibrated=False,
      online_sample_count=0,
      last_calibration_time=time.monotonic(),
    )

    self.calibrating = False
    self.calibration_complete = True

    cloudlog.info("Static calibration complete:")
    cloudlog.info(f"  Gyro bias: [{gyro_bias[0]:.6f}, {gyro_bias[1]:.6f}, {gyro_bias[2]:.6f}] rad/s")
    cloudlog.info(f"  Roll offset: {math.degrees(roll_offset):.2f} deg")
    cloudlog.info(f"  Pitch offset: {math.degrees(pitch_offset):.2f} deg")

  def get_progress(self) -> float:
    """Get calibration progress (0.0 to 1.0)."""
    if not self.calibrating:
      return 1.0 if self.calibration_complete else 0.0
    return len(self.accel_samples) / STATIC_SAMPLE_COUNT


class OnlineCalibrator:
  """
  Online IMU calibration.

  Learns calibration corrections from driving data over time.
  Uses straight-line driving segments to refine gyro bias.
  """

  def __init__(self, initial_cal: Optional[CalibrationData] = None):
    self.cal = initial_cal or CalibrationData()
    self.gyro_buffer: deque[list[float]] = deque(maxlen=100)
    self.accel_buffer: deque[list[float]] = deque(maxlen=100)
    self.speed_buffer: deque[float] = deque(maxlen=100)
    self._straight_segments = 0

  def update(self, accel: list[float], gyro: list[float], speed: float) -> CalibrationData:
    """
    Update online calibration with new sensor data.

    Args:
      accel: Accelerometer reading [x, y, z] in m/s^2
      gyro: Gyroscope reading [x, y, z] in rad/s
      speed: Vehicle speed in m/s

    Returns:
      Updated calibration data
    """
    self.gyro_buffer.append(gyro)
    self.accel_buffer.append(accel)
    self.speed_buffer.append(speed)

    # Only calibrate when we have enough data and vehicle is moving
    if len(self.gyro_buffer) < 50:
      return self.cal

    avg_speed = sum(self.speed_buffer) / len(self.speed_buffer)
    if avg_speed < ONLINE_MIN_SPEED:
      return self.cal

    # Check if we're driving straight (low gyro readings)
    recent_gyro = list(self.gyro_buffer)[-50:]
    gyro_magnitudes = [math.sqrt(g[0]**2 + g[1]**2 + g[2]**2) for g in recent_gyro]
    avg_gyro_magnitude = sum(gyro_magnitudes) / len(gyro_magnitudes)

    if avg_gyro_magnitude < ONLINE_MAX_GYRO:
      # We're driving straight - use this to refine gyro bias
      avg_gyro = [
        sum(g[i] for g in recent_gyro) / len(recent_gyro)
        for i in range(3)
      ]

      # Update gyro bias with exponential moving average
      for i in range(3):
        self.cal.gyro_bias[i] = (
          (1 - ONLINE_LEARNING_RATE) * self.cal.gyro_bias[i] +
          ONLINE_LEARNING_RATE * avg_gyro[i]
        )

      self._straight_segments += 1
      self.cal.online_sample_count = self._straight_segments
      self.cal.online_calibrated = self._straight_segments >= 10

      if self._straight_segments % 100 == 0:
        cloudlog.info(f"Online calibration update #{self._straight_segments}:")
        cloudlog.info(f"  Gyro bias: [{self.cal.gyro_bias[0]:.6f}, {self.cal.gyro_bias[1]:.6f}, {self.cal.gyro_bias[2]:.6f}]")

    return self.cal


def apply_calibration(
  accel: list[float],
  gyro: list[float],
  cal: CalibrationData
) -> tuple[list[float], list[float]]:
  """
  Apply calibration corrections to sensor data.

  Args:
    accel: Raw accelerometer reading [x, y, z] in m/s^2
    gyro: Raw gyroscope reading [x, y, z] in rad/s
    cal: Calibration data

  Returns:
    (corrected_accel, corrected_gyro)
  """
  # Apply gyro bias correction
  corrected_gyro = [
    gyro[i] - cal.gyro_bias[i]
    for i in range(3)
  ]

  # Apply roll/pitch rotation to accelerometer
  # This rotates the accelerometer readings to align with the vehicle frame
  if cal.roll_offset != 0 or cal.pitch_offset != 0:
    corrected_accel = _rotate_accel(accel, cal.roll_offset, cal.pitch_offset)
  else:
    corrected_accel = accel.copy()

  return corrected_accel, corrected_gyro


def _rotate_accel(accel: list[float], roll: float, pitch: float) -> list[float]:
  """
  Rotate accelerometer readings by roll and pitch angles.

  Uses rotation matrix: R = Ry(pitch) @ Rx(roll)
  """
  ax, ay, az = accel

  # Rotation around X axis (roll)
  cr, sr = math.cos(roll), math.sin(roll)
  ay_r = ay * cr - az * sr
  az_r = ay * sr + az * cr

  # Rotation around Y axis (pitch)
  cp, sp = math.cos(pitch), math.sin(pitch)
  ax_rp = ax * cp + az_r * sp
  az_rp = -ax * sp + az_r * cp

  return [ax_rp, ay_r, az_rp]


if __name__ == "__main__":
  # Test calibration
  import argparse

  parser = argparse.ArgumentParser(description="Test IMU calibration")
  parser.add_argument("--static", action="store_true", help="Run static calibration test")
  parser.add_argument("--show", action="store_true", help="Show current calibration")
  args = parser.parse_args()

  if args.show:
    cal = load_calibration()
    print("Current calibration:")
    print(f"  Static calibrated: {cal.static_calibrated}")
    print(f"  Online calibrated: {cal.online_calibrated}")
    print(f"  Gyro bias: {cal.gyro_bias}")
    print(f"  Roll offset: {math.degrees(cal.roll_offset):.2f} deg")
    print(f"  Pitch offset: {math.degrees(cal.pitch_offset):.2f} deg")
    print(f"  Online samples: {cal.online_sample_count}")

  elif args.static:
    print("Static calibration test with synthetic data...")
    calibrator = StaticCalibrator()
    calibrator.start()

    # Simulate tilted sensor (5 deg roll, 3 deg pitch)
    roll_rad = math.radians(5)
    pitch_rad = math.radians(3)

    # Gravity vector when tilted
    gx = -GRAVITY * math.sin(pitch_rad)
    gy = GRAVITY * math.sin(roll_rad) * math.cos(pitch_rad)
    gz = GRAVITY * math.cos(roll_rad) * math.cos(pitch_rad)

    for i in range(STATIC_SAMPLE_COUNT):
      # Add some noise
      noise = 0.01
      accel = [gx + noise * (i % 3 - 1), gy + noise * (i % 5 - 2), gz + noise * (i % 7 - 3)]
      gyro = [0.001 * (i % 3 - 1), 0.001 * (i % 5 - 2), 0.001 * (i % 7 - 3)]

      done = calibrator.add_sample(accel, gyro)
      if done:
        break

    if calibrator.result:
      print("\nCalibration result:")
      print(f"  Detected roll: {math.degrees(calibrator.result.roll_offset):.2f} deg (actual: 5.00 deg)")
      print(f"  Detected pitch: {math.degrees(calibrator.result.pitch_offset):.2f} deg (actual: 3.00 deg)")
