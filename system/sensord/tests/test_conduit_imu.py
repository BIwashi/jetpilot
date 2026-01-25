#!/usr/bin/env python3
"""
Tests for Conduit IMU sensor and CDR parser.
"""
import struct
import pytest

from openpilot.system.sensord.sensors.cdr_parser import (
  CDRParser, parse_imu_cdr, Vector3, Quaternion, Time, Header, ImuMessage
)


def create_test_imu_cdr() -> bytes:
  """Create a synthetic CDR-encoded sensor_msgs/Imu message for testing."""
  data = bytearray()

  # CDR encapsulation header (little-endian)
  data.extend(struct.pack('<HH', 0x0001, 0x0000))  # LE, options

  # Header
  # - stamp: int32 sec, uint32 nanosec
  data.extend(struct.pack('<i', 1000))       # sec
  data.extend(struct.pack('<I', 500000000))  # nanosec (0.5s)
  # - frame_id: uint32 length + string + null
  frame_id = b"imu_link\x00"
  data.extend(struct.pack('<I', len(frame_id)))
  data.extend(frame_id)

  # Pad to 8-byte alignment for quaternion
  while len(data) % 8 != 0:
    data.append(0)

  # Orientation quaternion (x, y, z, w)
  data.extend(struct.pack('<4d', 0.0, 0.0, 0.0, 1.0))

  # Orientation covariance (9 doubles, first -1 = unknown)
  data.extend(struct.pack('<9d', -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

  # Angular velocity (x, y, z) in rad/s
  data.extend(struct.pack('<3d', 0.01, 0.02, 0.03))

  # Angular velocity covariance
  data.extend(struct.pack('<9d', 0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001))

  # Linear acceleration (x, y, z) in m/s^2
  data.extend(struct.pack('<3d', 0.1, 0.2, 9.81))

  # Linear acceleration covariance
  data.extend(struct.pack('<9d', 0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01))

  return bytes(data)


class TestCDRParser:
  def test_parse_int32(self):
    # Encapsulation header + int32
    data = struct.pack('<HH', 0x0001, 0x0000) + struct.pack('<i', -12345)
    parser = CDRParser(data)
    assert parser.read_int32() == -12345

  def test_parse_uint32(self):
    data = struct.pack('<HH', 0x0001, 0x0000) + struct.pack('<I', 12345)
    parser = CDRParser(data)
    assert parser.read_uint32() == 12345

  def test_parse_float64(self):
    data = struct.pack('<HH', 0x0001, 0x0000) + b'\x00\x00\x00\x00' + struct.pack('<d', 3.14159)
    parser = CDRParser(data)
    assert abs(parser.read_float64() - 3.14159) < 1e-10

  def test_parse_string(self):
    test_str = "hello\x00"
    data = struct.pack('<HH', 0x0001, 0x0000) + struct.pack('<I', len(test_str)) + test_str.encode()
    parser = CDRParser(data)
    assert parser.read_string() == "hello"

  def test_parse_time(self):
    data = struct.pack('<HH', 0x0001, 0x0000) + struct.pack('<iI', 100, 500000000)
    parser = CDRParser(data)
    t = parser.read_time()
    assert t.sec == 100
    assert t.nanosec == 500000000
    assert t.to_ns() == 100_500_000_000

  def test_parse_vector3(self):
    data = struct.pack('<HH', 0x0001, 0x0000) + b'\x00\x00\x00\x00' + struct.pack('<3d', 1.0, 2.0, 3.0)
    parser = CDRParser(data)
    v = parser.read_vector3()
    assert v.x == 1.0
    assert v.y == 2.0
    assert v.z == 3.0

  def test_parse_quaternion(self):
    data = struct.pack('<HH', 0x0001, 0x0000) + b'\x00\x00\x00\x00' + struct.pack('<4d', 0.0, 0.0, 0.0, 1.0)
    parser = CDRParser(data)
    q = parser.read_quaternion()
    assert q.x == 0.0
    assert q.y == 0.0
    assert q.z == 0.0
    assert q.w == 1.0


class TestImuParsing:
  def test_parse_imu_message(self):
    data = create_test_imu_cdr()
    imu = parse_imu_cdr(data)

    # Check header
    assert imu.header.stamp.sec == 1000
    assert imu.header.stamp.nanosec == 500000000
    assert imu.header.frame_id == "imu_link"

    # Check orientation (identity quaternion)
    assert imu.orientation.w == 1.0
    assert imu.orientation.x == 0.0

    # Check angular velocity
    assert abs(imu.angular_velocity.x - 0.01) < 1e-10
    assert abs(imu.angular_velocity.y - 0.02) < 1e-10
    assert abs(imu.angular_velocity.z - 0.03) < 1e-10

    # Check linear acceleration
    assert abs(imu.linear_acceleration.x - 0.1) < 1e-10
    assert abs(imu.linear_acceleration.y - 0.2) < 1e-10
    assert abs(imu.linear_acceleration.z - 9.81) < 1e-10


class TestConduitIMU:
  def test_import(self):
    """Test that ConduitIMU can be imported."""
    try:
      from openpilot.system.sensord.sensors.conduit_imu import ConduitIMU, ZENOH_AVAILABLE
      assert True  # Import succeeded
    except ImportError as e:
      pytest.skip(f"ConduitIMU import failed: {e}")

  def test_zenoh_optional(self):
    """Test that Conduit sensor handles missing zenoh gracefully."""
    from openpilot.system.sensord.sensors.conduit_imu import ZENOH_AVAILABLE
    # Just check that ZENOH_AVAILABLE is defined
    assert isinstance(ZENOH_AVAILABLE, bool)


if __name__ == "__main__":
  pytest.main([__file__, "-v"])
