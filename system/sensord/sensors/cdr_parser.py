"""
Lightweight CDR (Common Data Representation) parser for ROS2 sensor_msgs/Imu.

This parser decodes CDR-encoded messages from Zenoh without requiring ROS2 or DDS dependencies.
Supports XCDR1 encapsulation as used by rmw_zenoh.
"""
import struct
from dataclasses import dataclass


@dataclass
class Vector3:
  """ROS2 geometry_msgs/Vector3"""
  x: float
  y: float
  z: float


@dataclass
class Quaternion:
  """ROS2 geometry_msgs/Quaternion"""
  x: float
  y: float
  z: float
  w: float


@dataclass
class Time:
  """ROS2 builtin_interfaces/Time"""
  sec: int
  nanosec: int

  def to_ns(self) -> int:
    return self.sec * 1_000_000_000 + self.nanosec


@dataclass
class Header:
  """ROS2 std_msgs/Header"""
  stamp: Time
  frame_id: str


@dataclass
class ImuMessage:
  """ROS2 sensor_msgs/Imu"""
  header: Header
  orientation: Quaternion
  orientation_covariance: tuple[float, ...]
  angular_velocity: Vector3
  angular_velocity_covariance: tuple[float, ...]
  linear_acceleration: Vector3
  linear_acceleration_covariance: tuple[float, ...]


class CDRParser:
  """
  Parser for CDR-encoded ROS2 messages.
  Handles XCDR1 encapsulation format used by Zenoh/DDS.
  """

  # CDR encapsulation identifiers
  CDR_LE = 0x0001  # Little-endian CDR
  CDR_BE = 0x0000  # Big-endian CDR

  def __init__(self, data: bytes):
    self.data = data
    self.offset = 0
    self.little_endian = True

    # Parse encapsulation header (4 bytes)
    if len(data) >= 4:
      encap = struct.unpack_from('<H', data, 0)[0]
      if encap == self.CDR_BE:
        self.little_endian = False
      # Skip 4-byte encapsulation header
      self.offset = 4

  def _endian_char(self) -> str:
    return '<' if self.little_endian else '>'

  def _align(self, alignment: int) -> None:
    """Align offset to the specified boundary."""
    remainder = self.offset % alignment
    if remainder != 0:
      self.offset += alignment - remainder

  def read_int32(self) -> int:
    self._align(4)
    value = struct.unpack_from(f'{self._endian_char()}i', self.data, self.offset)[0]
    self.offset += 4
    return value

  def read_uint32(self) -> int:
    self._align(4)
    value = struct.unpack_from(f'{self._endian_char()}I', self.data, self.offset)[0]
    self.offset += 4
    return value

  def read_float64(self) -> float:
    self._align(8)
    value = struct.unpack_from(f'{self._endian_char()}d', self.data, self.offset)[0]
    self.offset += 8
    return value

  def read_float64_array(self, count: int) -> tuple[float, ...]:
    """Read a fixed-size array of float64 values."""
    self._align(8)
    values = struct.unpack_from(f'{self._endian_char()}{count}d', self.data, self.offset)
    self.offset += 8 * count
    return values

  def read_string(self) -> str:
    """Read a CDR string (uint32 length + chars + null terminator)."""
    length = self.read_uint32()
    if length == 0:
      return ""
    # String includes null terminator in length
    string_bytes = self.data[self.offset:self.offset + length - 1]
    self.offset += length
    return string_bytes.decode('utf-8', errors='replace')

  def read_time(self) -> Time:
    """Read ROS2 builtin_interfaces/Time."""
    sec = self.read_int32()
    nanosec = self.read_uint32()
    return Time(sec=sec, nanosec=nanosec)

  def read_header(self) -> Header:
    """Read ROS2 std_msgs/Header."""
    stamp = self.read_time()
    frame_id = self.read_string()
    return Header(stamp=stamp, frame_id=frame_id)

  def read_vector3(self) -> Vector3:
    """Read ROS2 geometry_msgs/Vector3."""
    x = self.read_float64()
    y = self.read_float64()
    z = self.read_float64()
    return Vector3(x=x, y=y, z=z)

  def read_quaternion(self) -> Quaternion:
    """Read ROS2 geometry_msgs/Quaternion."""
    x = self.read_float64()
    y = self.read_float64()
    z = self.read_float64()
    w = self.read_float64()
    return Quaternion(x=x, y=y, z=z, w=w)

  def read_imu(self) -> ImuMessage:
    """Read ROS2 sensor_msgs/Imu message."""
    header = self.read_header()
    orientation = self.read_quaternion()
    orientation_covariance = self.read_float64_array(9)
    angular_velocity = self.read_vector3()
    angular_velocity_covariance = self.read_float64_array(9)
    linear_acceleration = self.read_vector3()
    linear_acceleration_covariance = self.read_float64_array(9)
    return ImuMessage(
      header=header,
      orientation=orientation,
      orientation_covariance=orientation_covariance,
      angular_velocity=angular_velocity,
      angular_velocity_covariance=angular_velocity_covariance,
      linear_acceleration=linear_acceleration,
      linear_acceleration_covariance=linear_acceleration_covariance,
    )


def parse_imu_cdr(data: bytes) -> ImuMessage:
  """Parse CDR-encoded sensor_msgs/Imu message."""
  parser = CDRParser(data)
  return parser.read_imu()


if __name__ == "__main__":
  # Basic module test
  print("CDR Parser module loaded successfully")
  print("Use parse_imu_cdr(data) to parse CDR-encoded IMU messages from Conduit")
