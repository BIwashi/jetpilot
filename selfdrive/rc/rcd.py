#!/usr/bin/env python3
# carControl を jetracer の PWM に橋渡しする簡易デーモン
# panda/CAN を使わず、擬似的な pandaStates と carState を流しつつ PWM を出力する

import json
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

from cereal import messaging, car, log
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params
from opendbc.car import scale_rot_inertia, scale_tire_stiffness

from jetracer.nvidia_racecar import NvidiaRacecar

try:
  import smbus  # type: ignore
except ImportError:  # pragma: no cover - platform dependent
  smbus = None


@dataclass
class RawParams:
  steer_left: int = 1000
  steer_center: int = 1500
  steer_right: int = 2000
  speed_front: int = 2000
  speed_stop: int = 1500
  speed_back: int = 1000


class RcReceiver:
  def __init__(self, bus_num: int, addr: int, reg: int, read_len: int = 12):
    self.addr = addr
    self.reg = reg
    self.read_len = read_len
    self.bus = smbus.SMBus(bus_num) if smbus is not None else None
    self.last: Optional[Tuple[int, int, int]] = None
    self.last_error_ts = 0.0

  def read(self) -> Optional[Tuple[int, int, int]]:
    if self.bus is None:
      return None
    try:
      data = self.bus.read_i2c_block_data(self.addr, self.reg, self.read_len)
      if len(data) < 12:
        return self.last
      steer_raw = _u32_be(data, 0)
      throttle_raw = _u32_be(data, 4)
      switch_raw = _u32_be(data, 8)
      self.last = (steer_raw, throttle_raw, switch_raw)
    except Exception as exc:
      now = time.monotonic()
      if now - self.last_error_ts > 2.0:
        cloudlog.warning(f"rcd: i2c read failed: {exc}")
        self.last_error_ts = now
    return self.last


def _clip(x: float, lo: float, hi: float) -> float:
  return max(lo, min(hi, x))


def _u32_be(data, offset: int) -> int:
  return (data[offset] << 24) | (data[offset + 1] << 16) | (data[offset + 2] << 8) | data[offset + 3]


def _map_linear(x: float, in_min: float, in_max: float, out_min: float, out_max: float) -> float:
  if in_max == in_min:
    return out_min
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def load_raw_params(path: str) -> RawParams:
  if not path:
    return RawParams()
  try:
    raw_path = Path(path).expanduser()
    with raw_path.open("r", encoding="utf-8") as f:
      data = json.load(f)
    return RawParams(
      steer_left=int(data["raw_steering"]["left"]),
      steer_center=int(data["raw_steering"]["center"]),
      steer_right=int(data["raw_steering"]["right"]),
      speed_front=int(data["raw_speed"]["front"]),
      speed_stop=int(data["raw_speed"]["stop"]),
      speed_back=int(data["raw_speed"]["back"]),
    )
  except Exception as exc:
    cloudlog.warning(f"rcd: failed to load raw params from {path}: {exc}")
    return RawParams()


def normalize_steer(raw: Optional[int], rp: RawParams) -> float:
  if raw is None:
    return 0.0
  if raw <= rp.steer_center:
    return _map_linear(raw, rp.steer_left, rp.steer_center, -1.0, 0.0)
  return _map_linear(raw, rp.steer_center, rp.steer_right, 0.0, 1.0)


def normalize_throttle(raw: Optional[int], rp: RawParams) -> float:
  if raw is None:
    return 0.0
  forward_is_high = rp.speed_front > rp.speed_stop
  if forward_is_high:
    if raw >= rp.speed_stop:
      return _map_linear(raw, rp.speed_stop, rp.speed_front, 0.0, 1.0)
    return _map_linear(raw, rp.speed_back, rp.speed_stop, -1.0, 0.0)
  if raw <= rp.speed_stop:
    return _map_linear(raw, rp.speed_stop, rp.speed_front, 0.0, 1.0)
  return _map_linear(raw, rp.speed_back, rp.speed_stop, -1.0, 0.0)


def build_car_params() -> car.CarParams:
  """最小限の CarParams を組み立てて Params に書き込む。"""
  cp = car.CarParams.new_message()
  # 既存のinterfaceに確実にある指紋にしておく（例: HYUNDAI SONATA 2019）
  car_fp = os.getenv("RCD_CAR_FINGERPRINT", "LEXUS_RX_TSS2")
  car_brand = os.getenv("RCD_CAR_BRAND", "toyota")
  cp.carFingerprint = car_fp
  cp.brand = car_brand
  cp.mass = 3.0
  cp.wheelbase = 0.26
  cp.centerToFront = cp.wheelbase * 0.5
  cp.steerRatio = 1.0
  cp.steerRatioRear = 0.0
  cp.steerActuatorDelay = 0.0
  cp.longitudinalActuatorDelay = 0.0
  cp.openpilotLongitudinalControl = bool(int(os.getenv("RCD_LONG_CONTROL", "0")))
  cp.pcmCruise = True
  cp.notCar = bool(int(os.getenv("RCD_NOT_CAR", "1")))
  cp.minEnableSpeed = -1.0
  cp.minSteerSpeed = 0.0
  cp.steerAtStandstill = True
  cp.steerControlType = car.CarParams.SteerControlType.angle
  cp.tireStiffnessFactor = 1.0
  cp.rotationalInertia = scale_rot_inertia(cp.mass, cp.wheelbase)
  cp.tireStiffnessFront, cp.tireStiffnessRear = scale_tire_stiffness(
    cp.mass, cp.wheelbase, cp.centerToFront, cp.tireStiffnessFactor
  )

  safety = car.CarParams.SafetyConfig.new_message()
  safety.safetyModel = car.CarParams.SafetyModel.noOutput
  cp.safetyConfigs = [safety]

  return cp


def publish_car_params_once(pm: messaging.PubMaster, params: Params, cp_reader: car.CarParams, cp_bytes: bytes) -> None:
  params.put("CarParams", cp_bytes)
  params.put_nonblocking("CarParamsCache", cp_bytes)
  params.put_nonblocking("CarParamsPersistent", cp_bytes)

  msg = messaging.new_message("carParams")
  msg.valid = True
  msg.carParams = cp_reader
  pm.send("carParams", msg)


def make_panda_states_msg(controls_allowed: bool):
  """pandaStates を1要素だけ入れたメッセージを作る。"""
  msg = messaging.new_message("pandaStates", 1)
  msg.valid = True
  ps = msg.pandaStates[0]
  ps.pandaType = log.PandaState.PandaType.redPanda
  ps.ignitionLine = True
  ps.ignitionCan = True
  ps.controlsAllowed = controls_allowed
  ps.powerSaveEnabled = False
  ps.faultStatus = 0
  ps.safetyModel = car.CarParams.SafetyModel.noOutput
  ps.safetyParam = 0
  return msg


def make_accel_event(log_mono_time_ns: int) -> log.SensorEventData:
  event = log.SensorEventData.new_message()
  event.timestamp = log_mono_time_ns
  event.version = 1
  event.sensor = 1  # SENSOR_ACCELEROMETER
  event.type = 1    # SENSOR_TYPE_ACCELEROMETER
  event.source = log.SensorEventData.SensorSource.lsm6ds3
  accel = event.init("acceleration")
  accel.v = [0.0, 0.0, 0.0]
  accel.status = 1
  return event


def make_gyro_event(log_mono_time_ns: int) -> log.SensorEventData:
  event = log.SensorEventData.new_message()
  event.timestamp = log_mono_time_ns
  event.version = 2
  event.sensor = 5  # SENSOR_GYRO_UNCALIBRATED
  event.type = 16   # SENSOR_TYPE_GYROSCOPE_UNCALIBRATED
  event.source = log.SensorEventData.SensorSource.lsm6ds3
  gyro = event.init("gyroUncalibrated")
  gyro.v = [0.0, 0.0, 0.0]
  gyro.status = 1
  return event


def make_live_calibration_msg():
  msg = messaging.new_message("liveCalibration", valid=True)
  lc = msg.liveCalibration
  lc.calStatus = log.LiveCalibrationData.Status.calibrated
  lc.calPerc = 100
  lc.calCycle = 0
  lc.validBlocks = 1
  lc.rpyCalib = [0.0, 0.0, 0.0]
  lc.rpyCalibSpread = [0.0, 0.0, 0.0]
  lc.wideFromDeviceEuler = [0.0, 0.0, 0.0]
  lc.height = [0.0]
  return msg


def make_peripheral_state_msg():
  msg = messaging.new_message("peripheralState", valid=True)
  ps = msg.peripheralState
  ps.pandaType = log.PandaState.PandaType.redPanda
  ps.voltage = 12000
  ps.current = 0
  ps.fanSpeedRpm = 0
  return msg


def _fill_xyz(meas, x: float = 0.0, y: float = 0.0, z: float = 0.0, std: float = 0.1) -> None:
  meas.x = x
  meas.y = y
  meas.z = z
  meas.xStd = std
  meas.yStd = std
  meas.zStd = std
  meas.valid = True


def make_live_pose_msg():
  msg = messaging.new_message("livePose", valid=True)
  lp = msg.livePose
  _fill_xyz(lp.orientationNED)
  _fill_xyz(lp.velocityDevice)
  _fill_xyz(lp.accelerationDevice)
  _fill_xyz(lp.angularVelocityDevice)
  lp.inputsOK = True
  lp.posenetOK = True
  lp.sensorsOK = True
  return msg


def make_live_parameters_msg(cp: car.CarParams):
  msg = messaging.new_message("liveParameters", valid=True)
  lp = msg.liveParameters
  lp.valid = True
  lp.sensorValid = True
  lp.posenetValid = True
  lp.angleOffsetDeg = 0.0
  lp.angleOffsetAverageDeg = 0.0
  lp.stiffnessFactor = 1.0
  lp.steerRatio = float(cp.steerRatio)
  lp.posenetSpeed = 0.0
  lp.angleOffsetFastStd = 0.1
  lp.angleOffsetAverageStd = 0.1
  lp.stiffnessFactorStd = 0.1
  lp.steerRatioStd = 0.1
  lp.roll = 0.0
  return msg


def make_live_delay_msg():
  msg = messaging.new_message("liveDelay", valid=True)
  ld = msg.liveDelay
  ld.lateralDelay = 0.1
  ld.validBlocks = 1
  ld.status = log.LiveDelayData.Status.estimated
  ld.lateralDelayEstimate = ld.lateralDelay
  ld.lateralDelayEstimateStd = 0.0
  ld.calPerc = 100
  ld.points = [0.0]
  return msg


def make_live_torque_parameters_msg():
  msg = messaging.new_message("liveTorqueParameters", valid=True)
  lt = msg.liveTorqueParameters
  lt.liveValid = True
  lt.latAccelFactorRaw = 0.0
  lt.latAccelOffsetRaw = 0.0
  lt.frictionCoefficientRaw = 0.0
  lt.latAccelFactorFiltered = 0.0
  lt.latAccelOffsetFiltered = 0.0
  lt.frictionCoefficientFiltered = 0.0
  lt.totalBucketPoints = 0.0
  lt.decay = 0.0
  lt.maxResets = 0.0
  lt.points = []
  lt.version = 0
  lt.useParams = False
  lt.calPerc = 100
  return msg


def make_radar_state_msg():
  msg = messaging.new_message("radarState", valid=True)
  rs = msg.radarState
  rs.radarErrors.canError = False
  rs.radarErrors.radarFault = False
  rs.radarErrors.wrongConfig = False
  rs.radarErrors.radarUnavailableTemporary = False
  rs.leadOne.status = False
  rs.leadTwo.status = False
  return msg


def make_gps_location_msg():
  msg = messaging.new_message("gpsLocation", valid=True)
  gl = msg.gpsLocation
  gl.flags = 1
  gl.latitude = 0.0
  gl.longitude = 0.0
  gl.altitude = 0.0
  gl.speed = 0.0
  gl.bearingDeg = 0.0
  gl.unixTimestampMillis = int(time.time() * 1000)
  gl.horizontalAccuracy = 1.0
  gl.verticalAccuracy = 1.0
  gl.speedAccuracy = 0.1
  gl.bearingAccuracyDeg = 0.1
  return msg


def make_car_state_msg(v_ego: float, a_ego: float, steer_deg: float, steer_rate: float, standstill: bool,
                       cruise_enabled: bool, v_cruise_kph: float):
  msg = messaging.new_message("carState")
  msg.valid = True

  cs = msg.carState
  cs.vEgo = v_ego
  cs.aEgo = a_ego
  cs.vEgoRaw = v_ego
  cs.steeringAngleDeg = steer_deg
  cs.canValid = True
  cs.cruiseState.enabled = cruise_enabled
  cs.cruiseState.available = True
  cs.cruiseState.speed = v_ego
  cs.cruiseState.speedCluster = v_ego
  cs.cruiseState.standstill = standstill
  cs.vCruise = v_cruise_kph
  cs.vCruiseCluster = v_cruise_kph
  cs.gearShifter = car.CarState.GearShifter.drive
  cs.steeringRateDeg = steer_rate
  cs.yawRate = 0.0
  cs.standstill = standstill
  cs.wheelSpeeds.fl = v_ego
  cs.wheelSpeeds.fr = v_ego
  cs.wheelSpeeds.rl = v_ego
  cs.wheelSpeeds.rr = v_ego
  return msg


def map_actuators_to_pwm(actuators: car.CarControl.Actuators, steer_max_deg: float, accel_gain: float) -> Tuple[float, float]:
  """carControl のアクチュエータを -1〜1 に正規化して返す。"""
  steer_cmd = 0.0
  if steer_max_deg > 0:
    steer_cmd = _clip(actuators.steeringAngleDeg / steer_max_deg, -1.0, 1.0)

  # accel[m/s^2] を適当なゲインでスロットルにマップ（負値で後退とみなす）
  throttle_cmd = _clip(actuators.accel * accel_gain, -1.0, 1.0)
  return steer_cmd, throttle_cmd


def main():
  # 環境変数でパラメータ調整可能にする
  car_type = os.getenv("RCD_RACECAR_TYPE", "TT02")
  steer_max_deg = float(os.getenv("RCD_STEER_MAX_DEG", "25.0"))
  accel_gain = float(os.getenv("RCD_ACCEL_GAIN", "0.1"))
  v_ego_scale = float(os.getenv("RCD_VEGO_SCALE_MPS", "1.0"))
  rate_hz = float(os.getenv("RCD_RATE_HZ", "100.0"))
  i2c_bus = int(os.getenv("RCD_I2C_BUS", "7"))
  i2c_addr = int(os.getenv("RCD_I2C_ADDR", "0x08"), 0)
  i2c_reg = int(os.getenv("RCD_I2C_REG", "0x01"), 0)
  raw_params_path = os.getenv("RCD_RAW_PARAMS", "/home/jetson/jetracer/notebooks/raw_params.json")
  switch_threshold = int(os.getenv("RCD_AI_SWITCH_THRESHOLD_US", "1500"))
  switch_invert = bool(int(os.getenv("RCD_AI_SWITCH_INVERT", "0")))
  use_op_throttle = bool(int(os.getenv("RCD_USE_OP_THROTTLE", "0")))

  cloudlog.info(
    "rcd starting: type=%s steer_max_deg=%.2f accel_gain=%.3f rate_hz=%.1f i2c_bus=%d i2c_addr=0x%x",
    car_type, steer_max_deg, accel_gain, rate_hz, i2c_bus, i2c_addr,
  )

  params = Params()
  publish_imu = bool(int(os.getenv("RCD_PUBLISH_IMU", "1")))
  publish_live_calib = bool(int(os.getenv("RCD_PUBLISH_LIVE_CALIB", "0")))
  publish_peripheral_state = bool(int(os.getenv("RCD_PUBLISH_PERIPHERAL_STATE", "1")))
  publish_fake_sensors = bool(int(os.getenv("RCD_PUBLISH_FAKE_SENSORS", "0")))
  pub_services = ["pandaStates", "carState", "carParams", "carOutput"]
  if publish_imu:
    pub_services += ["accelerometer", "gyroscope"]
  if publish_live_calib:
    pub_services.append("liveCalibration")
  if publish_peripheral_state:
    pub_services.append("peripheralState")
  if publish_fake_sensors:
    pub_services += [
      "livePose",
      "liveParameters",
      "liveDelay",
      "liveTorqueParameters",
      "radarState",
      "gpsLocation",
    ]
  pm = messaging.PubMaster(pub_services)
  sm = messaging.SubMaster(["carControl"])

  racecar = NvidiaRacecar(type=car_type)
  raw_params = load_raw_params(raw_params_path)
  rc_reader = RcReceiver(i2c_bus, i2c_addr, i2c_reg)

  cp = build_car_params()
  cp_bytes = cp.to_bytes()
  cp_reader = cp.as_reader()
  publish_car_params_once(pm, params, cp_reader, cp_bytes)
  params.put("PandaSignatures", b"fake_panda_signature")

  rk = Ratekeeper(rate_hz, print_delay_threshold=None)
  last_cp_pub = time.monotonic()
  last_panda_pub = time.monotonic()
  last_calib_pub = time.monotonic()
  last_periph_pub = time.monotonic()
  last_live_pose_pub = time.monotonic()
  last_live_params_pub = time.monotonic()
  last_live_delay_pub = time.monotonic()
  last_live_torque_pub = time.monotonic()
  last_radar_pub = time.monotonic()
  last_gps_pub = time.monotonic()
  last_v_ego = 0.0
  last_steer_deg = 0.0
  prev_ai_enabled = False
  throttle_hold = 0.0

  while True:
    sm.update(0)

    rc_raw = rc_reader.read()
    steer_raw, throttle_raw, switch_raw = rc_raw if rc_raw is not None else (None, None, None)
    manual_steer = _clip(normalize_steer(steer_raw, raw_params), -1.0, 1.0)
    manual_throttle = _clip(normalize_throttle(throttle_raw, raw_params), -1.0, 1.0)

    ai_enabled = False
    if switch_raw is not None:
      ai_enabled = switch_raw > switch_threshold
      if switch_invert:
        ai_enabled = not ai_enabled

    if ai_enabled and not prev_ai_enabled:
      throttle_hold = manual_throttle
    prev_ai_enabled = ai_enabled

    steer_cmd = manual_steer
    throttle_cmd = manual_throttle
    cc = sm["carControl"] if sm.all_valid(["carControl"]) else None

    if ai_enabled and cc is not None and cc.latActive:
      steer_cmd = map_actuators_to_pwm(cc.actuators, steer_max_deg, accel_gain)[0]

    if ai_enabled:
      if use_op_throttle and cc is not None and cc.longActive:
        throttle_cmd = map_actuators_to_pwm(cc.actuators, steer_max_deg, accel_gain)[1]
      else:
        throttle_cmd = throttle_hold

    racecar.steering = steer_cmd
    racecar.throttle = throttle_cmd

    v_ego = max(0.0, manual_throttle) * v_ego_scale
    a_ego = (v_ego - last_v_ego) * rate_hz
    last_v_ego = v_ego
    steer_deg = steer_cmd * steer_max_deg
    steer_rate = (steer_deg - last_steer_deg) * rate_hz
    last_steer_deg = steer_deg

    standstill = abs(v_ego) < 0.05
    v_cruise_kph = v_ego * 3.6

    # IMU (dummy) at loop rate
    if publish_imu:
      acc_msg = messaging.new_message("accelerometer", valid=True)
      acc_msg.accelerometer = make_accel_event(acc_msg.logMonoTime)
      pm.send("accelerometer", acc_msg)

      gyro_msg = messaging.new_message("gyroscope", valid=True)
      gyro_msg.gyroscope = make_gyro_event(gyro_msg.logMonoTime)
      pm.send("gyroscope", gyro_msg)

    # pandaStates at ~10Hz
    now = time.monotonic()
    if now - last_panda_pub >= 0.1:
      pm.send("pandaStates", make_panda_states_msg(ai_enabled))
      last_panda_pub = now

    # peripheralState at ~2Hz
    if publish_peripheral_state and now - last_periph_pub >= 0.5:
      pm.send("peripheralState", make_peripheral_state_msg())
      last_periph_pub = now

    # liveCalibration at ~5Hz (optional to avoid publisher conflicts)
    if publish_live_calib and now - last_calib_pub >= 0.2:
      pm.send("liveCalibration", make_live_calibration_msg())
      last_calib_pub = now

    if publish_fake_sensors:
      if now - last_live_pose_pub >= 0.05:
        pm.send("livePose", make_live_pose_msg())
        last_live_pose_pub = now
      if now - last_live_params_pub >= 0.05:
        pm.send("liveParameters", make_live_parameters_msg(cp))
        last_live_params_pub = now
      if now - last_live_delay_pub >= 0.25:
        pm.send("liveDelay", make_live_delay_msg())
        last_live_delay_pub = now
      if now - last_live_torque_pub >= 0.25:
        pm.send("liveTorqueParameters", make_live_torque_parameters_msg())
        last_live_torque_pub = now
      if now - last_radar_pub >= 0.05:
        pm.send("radarState", make_radar_state_msg())
        last_radar_pub = now
      if now - last_gps_pub >= 1.0:
        pm.send("gpsLocation", make_gps_location_msg())
        last_gps_pub = now

    # carState を擬似送信（RC入力ベース）
    cs_msg = make_car_state_msg(v_ego, a_ego, steer_deg, steer_rate, standstill, ai_enabled, v_cruise_kph)
    pm.send("carState", cs_msg)

    # carOutput は実際に出した値を反映
    co_msg = messaging.new_message("carOutput")
    co_msg.valid = True
    co_msg.carOutput.actuatorsOutput.steeringAngleDeg = steer_deg
    co_msg.carOutput.actuatorsOutput.torque = 0.0
    pm.send("carOutput", co_msg)

    # carParams はたまに再送（controlsd 初期化漏れを防ぐため）
    if now - last_cp_pub > 5.0:
      publish_car_params_once(pm, params, cp_reader, cp_bytes)
      last_cp_pub = now

    rk.keep_time()


if __name__ == "__main__":
  main()
