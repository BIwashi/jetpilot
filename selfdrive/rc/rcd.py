#!/usr/bin/env python3
# carControl を jetracer の PWM に橋渡しする簡易デーモン
# panda/CAN を使わず、擬似的な pandaStates と carState を流しつつ PWM を出力する

import json
import os
import time
from cereal import messaging, car, log
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params

import smbus
from jetracer.nvidia_racecar import NvidiaRacecar

# I2C でプロポからの PWM 値を読み取るためのアドレス
PROPO_I2C_ADDR = 0x08


def _clip(x: float, lo: float, hi: float) -> float:
  return max(lo, min(hi, x))


class PropoReader:
  """プロポからの PWM 入力を I2C 経由で読み取る"""

  def __init__(self, i2c_bus: int = 7, raw_params_path: str = None):
    self.addr = PROPO_I2C_ADDR
    self.bus = smbus.SMBus(i2c_bus)
    cloudlog.info(f"PropoReader: I2C bus {i2c_bus}, addr 0x{self.addr:02x}")

    # デフォルトのキャリブレーション値
    self.steering_left = 1524
    self.steering_center = 1524
    self.steering_right = 1980
    self.speed_front = 1047
    self.speed_stop = 1507
    self.speed_back = 1950

    # raw_params.json からキャリブレーション値を読み込む
    if raw_params_path and os.path.exists(raw_params_path):
      try:
        with open(raw_params_path) as f:
          params = json.load(f)
          self.steering_left = params["raw_steering"]["left"]
          self.steering_center = params["raw_steering"]["center"]
          self.steering_right = params["raw_steering"]["right"]
          self.speed_front = params["raw_speed"]["front"]
          self.speed_stop = params["raw_speed"]["stop"]
          self.speed_back = params["raw_speed"]["back"]
          cloudlog.info(f"PropoReader: loaded params from {raw_params_path}")
      except Exception as e:
        cloudlog.warning(f"PropoReader: failed to load {raw_params_path}: {e}")

  def read_raw(self) -> tuple[int, int]:
    """プロポから生の PWM 値を読み取る (steering, throttle)"""
    # notebook と同じく 12 バイト読み取る
    data = self.bus.read_i2c_block_data(self.addr, 0x01, 12)
    raw_steering = data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3]
    raw_throttle = data[4] << 24 | data[5] << 16 | data[6] << 8 | data[7]
    return raw_steering, raw_throttle

  def get_throttle_normalized(self) -> float:
    """スロットル開度を -1.0 (後退) ~ 0.0 (停止) ~ 1.0 (前進) で返す"""
    _, raw_throttle = self.read_raw()

    if raw_throttle <= self.speed_front:
      # 前進最大
      return 1.0
    elif raw_throttle >= self.speed_back:
      # 後退最大
      return -1.0
    elif raw_throttle < self.speed_stop:
      # 前進 (front ~ stop)
      return (self.speed_stop - raw_throttle) / (self.speed_stop - self.speed_front)
    elif raw_throttle > self.speed_stop:
      # 後退 (stop ~ back)
      return -(raw_throttle - self.speed_stop) / (self.speed_back - self.speed_stop)
    else:
      return 0.0

  def get_steering_normalized(self) -> float:
    """ステアリング角度を -1.0 (左) ~ 0.0 (中央) ~ 1.0 (右) で返す"""
    raw_steering, _ = self.read_raw()

    if raw_steering <= self.steering_left:
      return -1.0
    elif raw_steering >= self.steering_right:
      return 1.0
    elif raw_steering < self.steering_center:
      return -(self.steering_center - raw_steering) / (self.steering_center - self.steering_left)
    elif raw_steering > self.steering_center:
      return (raw_steering - self.steering_center) / (self.steering_right - self.steering_center)
    else:
      return 0.0


def build_car_params() -> car.CarParams:
  """最小限の CarParams を組み立てて Params に書き込む。"""
  cp = car.CarParams.new_message()
  # COMMA_BODY を使う（interfaces に登録されている fingerprint が必要）
  car_fp = os.getenv("RCD_CAR_FINGERPRINT", "COMMA_BODY")
  cp.carFingerprint = car_fp
  cp.brand = "body"

  # 車両物理パラメータ（ラジコン向け）
  cp.mass = 3.0  # kg
  cp.wheelbase = 0.26  # m
  cp.centerToFront = cp.wheelbase * 0.5
  cp.steerRatio = 1.0  # 直接舵角制御なので1:1

  # VehicleModel が必要とするパラメータ（ゼロ除算回避）
  cp.tireStiffnessFront = 10000.0  # N/rad (適当な正の値)
  cp.tireStiffnessRear = 10000.0   # N/rad
  cp.rotationalInertia = 0.1       # kg*m^2

  # ステアリング制御設定
  cp.steerControlType = car.CarParams.SteerControlType.angle  # 直接舵角制御
  cp.steerActuatorDelay = 0.0
  cp.steerLimitTimer = 0.0
  cp.steerLimitAlert = False

  # 縦方向制御
  cp.openpilotLongitudinalControl = True
  cp.pcmCruise = False
  cp.vEgoStopping = 0.1
  cp.vEgoStarting = 0.1
  cp.stoppingDecelRate = 0.5

  # 横方向制御チューニング（angle制御用）
  # angle 制御では lateralTuning は使われないが、念のため初期化
  cp.lateralTuning.init('pid')
  cp.lateralTuning.pid.kpV = [0.0]
  cp.lateralTuning.pid.kiV = [0.0]
  cp.lateralTuning.pid.kf = 0.0

  # 安全設定
  safety = car.CarParams.SafetyConfig.new_message()
  safety.safetyModel = car.CarParams.SafetyModel.noOutput
  cp.safetyConfigs = [safety]

  # その他
  cp.notCar = False  # selfdrived で notCar 扱いにしない
  cp.passive = False

  return cp


def publish_car_params_once(pm: messaging.PubMaster, params: Params, cp_reader: car.CarParams, cp_bytes: bytes) -> None:
  params.put("CarParams", cp_bytes)
  params.put_nonblocking("CarParamsCache", cp_bytes)
  params.put_nonblocking("CarParamsPersistent", cp_bytes)

  msg = messaging.new_message("carParams")
  msg.valid = True
  msg.carParams = cp_reader
  pm.send("carParams", msg)


def make_panda_states_msg():
  """pandaStates を1要素だけ入れたメッセージを作る。"""
  msg = messaging.new_message("pandaStates", 1)
  msg.valid = True  # SubMaster の valid チェックに必要
  ps = msg.pandaStates[0]
  ps.pandaType = log.PandaState.PandaType.redPanda
  ps.ignitionLine = True
  ps.ignitionCan = True
  ps.controlsAllowed = True
  ps.powerSaveEnabled = False
  ps.faultStatus = 0
  ps.safetyModel = car.CarParams.SafetyModel.noOutput
  ps.safetyParam = 0
  return msg


def make_car_state_msg(v_ego: float, steer_deg: float, v_cruise_kph: float = 10.0):  # ラジコン向け: 10 km/h
  msg = messaging.new_message("carState")
  msg.valid = True

  cs = msg.carState
  cs.vEgo = v_ego
  cs.vEgoRaw = v_ego
  cs.vEgoCluster = v_ego
  cs.steeringAngleDeg = steer_deg
  cs.canValid = True

  # クルーズ状態
  cs.cruiseState.enabled = True
  cs.cruiseState.available = True
  cs.cruiseState.speed = v_cruise_kph / 3.6  # m/s
  cs.cruiseState.standstill = False

  cs.gearShifter = car.CarState.GearShifter.drive
  cs.steeringRateDeg = 0.0
  cs.yawRate = 0.0
  cs.standstill = v_ego < 0.01
  cs.steeringPressed = False
  cs.steerFaultTemporary = False
  cs.steerFaultPermanent = False
  cs.brakePressed = False
  cs.gasPressed = False
  cs.aEgo = 0.0

  # vCruise (controlsd / selfdrived が参照)
  cs.vCruise = v_cruise_kph
  cs.vCruiseCluster = v_cruise_kph

  return msg


def make_car_output_msg(steer_deg: float, accel: float):
  """carOutput を作成。controlsd が steer_limited_by_safety を判定するのに必要。"""
  msg = messaging.new_message("carOutput")
  msg.valid = True

  co = msg.carOutput
  co.actuatorsOutput.steeringAngleDeg = steer_deg
  co.actuatorsOutput.accel = accel

  return msg


def make_sensor_msg(sensor_type: str, log_mono_time: int):
  """擬似センサーデータを作成。selfdrived の sensorDataInvalid チェック回避用。"""
  msg = messaging.new_message(sensor_type)
  msg.valid = True

  sensor = getattr(msg, sensor_type)
  sensor.version = 1
  sensor.sensor = 0
  sensor.type = 0
  # logMonoTime に近い値を使う（100ms 以内でないと警告が出る）
  sensor.timestamp = log_mono_time
  sensor.source = log.SensorEventData.SensorSource.lsm6ds3

  if sensor_type == "accelerometer":
    # union フィールドは init が必要
    accel = sensor.init("acceleration")
    accel.v = [0.0, 0.0, 9.81]  # 重力加速度 (静止状態)
    accel.status = 1
  elif sensor_type == "gyroscope":
    gyro = sensor.init("gyro")
    gyro.v = [0.0, 0.0, 0.0]  # 静止状態
    gyro.status = 1

  return msg


def map_actuators_to_pwm(actuators: car.CarControl.Actuators, steer_max_deg: float, accel_gain: float) -> tuple[float, float]:
  """carControl のアクチュエータを -1〜1 に正規化して返す。"""
  steer_cmd = 0.0
  if steer_max_deg > 0:
    steer_cmd = _clip(actuators.steeringAngleDeg / steer_max_deg, -1.0, 1.0)

  # accel[m/s^2] を適当なゲインでスロットルにマップ（負値で後退とみなす）
  throttle_cmd = _clip(actuators.accel * accel_gain, -1.0, 1.0)
  return steer_cmd, throttle_cmd


def main():
  # 環境変数でパラメータ調整可能にする
  car_type = os.getenv("RCD_RACECAR_TYPE", "TT02")  # デフォルトを TT02 に
  steer_max_deg = float(os.getenv("RCD_STEER_MAX_DEG", "30.0"))
  accel_gain = float(os.getenv("RCD_ACCEL_GAIN", "0.1"))
  # TT02 シャーシの最大速度 (ラジコン向けスケール)
  # 最大速度 (スロットル 100% 時の速度) [m/s]
  # 実際のラジコンは約 3-5 m/s 程度
  v_max = float(os.getenv("RCD_VMAX_MPS", "3.0"))  # 3 m/s = 10.8 km/h
  rate_hz = float(os.getenv("RCD_RATE_HZ", "100.0"))
  i2c_bus = int(os.getenv("RCD_I2C_BUS", "7"))  # Jetson Orin Nano は 7

  # raw_params.json のパス
  raw_params_path = os.getenv("RCD_RAW_PARAMS", "/home/jetson/workspace/jetpilot/jetracer_repo/notebooks/raw_params.json")

  cloudlog.info(f"rcd starting with type={car_type}, steer_max_deg={steer_max_deg}, v_max={v_max}, rate_hz={rate_hz}")

  params = Params()
  pm = messaging.PubMaster(["pandaStates", "carState", "carParams", "carOutput", "accelerometer", "gyroscope"])
  sm = messaging.SubMaster(["carControl", "selfdriveState"])

  racecar = NvidiaRacecar(type=car_type)

  # プロポからの入力を読み取るためのリーダー
  propo = PropoReader(i2c_bus=i2c_bus, raw_params_path=raw_params_path)

  cp = build_car_params()
  cp_bytes = cp.to_bytes()
  cp_reader = cp.as_reader()
  publish_car_params_once(pm, params, cp_reader, cp_bytes)
  params.put("PandaSignatures", b"fake_panda_signature")

  rk = Ratekeeper(rate_hz, print_delay_threshold=None)
  last_cp_pub = time.monotonic()

  # 出力した actuator 値を追跡（carOutput 用）
  last_steer_deg = 0.0
  last_accel = 0.0

  while True:
    sm.update(0)

    # プロポからの実際のスロットル入力を読み取って速度を推定
    propo_throttle = propo.get_throttle_normalized()
    propo_steering = propo.get_steering_normalized()

    # スロットルから速度を推定 (前進のみ、後退は 0 扱い)
    if propo_throttle > 0.05:
      current_v_ego = v_max * propo_throttle
    else:
      current_v_ego = 0.0

    # openpilot が有効な場合は carControl に従う
    # 無効な場合はプロポからの入力をそのままラジコンに送る
    ss = sm["selfdriveState"]
    if ss.enabled and sm.all_valid(["carControl"]):
      cc = sm["carControl"]
      steer_cmd, throttle_cmd = map_actuators_to_pwm(cc.actuators, steer_max_deg, accel_gain)
      racecar.steering = steer_cmd
      racecar.throttle = throttle_cmd

      # carOutput 用に実際に出力した値を記録
      last_steer_deg = cc.actuators.steeringAngleDeg
      last_accel = cc.actuators.accel
    else:
      # openpilot 無効時はプロポからの入力をパススルー
      racecar.steering = propo_steering
      racecar.throttle = propo_throttle

      # プロポの入力をそのまま carOutput に反映
      last_steer_deg = propo_steering * steer_max_deg
      last_accel = propo_throttle * (1.0 / accel_gain) if accel_gain > 0 else 0.0

    # pandaStates を毎ループ送る
    pm.send("pandaStates", make_panda_states_msg())

    # carState を擬似送信
    pm.send("carState", make_car_state_msg(current_v_ego, last_steer_deg))

    # carOutput を毎ループ送る（controlsd が steer_limited_by_safety を判定するのに必要）
    pm.send("carOutput", make_car_output_msg(last_steer_deg, last_accel))

    # 擬似センサーデータを送信（selfdrived の sensorDataInvalid チェック回避）
    # logMonoTime に近いタイムスタンプを使う
    now_nanos = int(time.monotonic() * 1e9)
    pm.send("accelerometer", make_sensor_msg("accelerometer", now_nanos))
    pm.send("gyroscope", make_sensor_msg("gyroscope", now_nanos))

    # carParams はたまに再送（controlsd 初期化漏れを防ぐため）
    now = time.monotonic()
    if now - last_cp_pub > 5.0:
      publish_car_params_once(pm, params, cp_reader, cp_bytes)
      last_cp_pub = now

    rk.keep_time()


if __name__ == "__main__":
  main()
