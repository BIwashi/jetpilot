#!/usr/bin/env python3
# carControl を jetracer の PWM に橋渡しする簡易デーモン
# panda/CAN を使わず、擬似的な pandaStates と carState を流しつつ PWM を出力する

import os
import time
from cereal import messaging, car, log
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params

from jetracer.nvidia_racecar import NvidiaRacecar


def _clip(x: float, lo: float, hi: float) -> float:
  return max(lo, min(hi, x))


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


def make_car_state_msg(v_ego: float, steer_deg: float, v_cruise_kph: float = 30.0):
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


def make_sensor_msg(sensor_type: str):
  """擬似センサーデータを作成。selfdrived の sensorDataInvalid チェック回避用。"""
  msg = messaging.new_message(sensor_type)
  msg.valid = True

  sensor = getattr(msg, sensor_type)
  sensor.version = 1
  sensor.sensor = 0
  sensor.type = 0
  sensor.timestamp = int(time.time() * 1e9)
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
  car_type = os.getenv("RCD_RACECAR_TYPE", "OPTION")
  steer_max_deg = float(os.getenv("RCD_STEER_MAX_DEG", "30.0"))
  accel_gain = float(os.getenv("RCD_ACCEL_GAIN", "0.1"))
  v_ego_stub = float(os.getenv("RCD_VEGO_MPS", "1.5"))
  rate_hz = float(os.getenv("RCD_RATE_HZ", "100.0"))

  cloudlog.info(f"rcd starting with type={car_type}, steer_max_deg={steer_max_deg}, accel_gain={accel_gain}, rate_hz={rate_hz}")

  params = Params()
  pm = messaging.PubMaster(["pandaStates", "carState", "carParams", "carOutput", "accelerometer", "gyroscope"])
  sm = messaging.SubMaster(["carControl"])

  racecar = NvidiaRacecar(type=car_type)

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

    # carControl が来ていれば PWM を更新
    if sm.all_valid(["carControl"]):
      cc = sm["carControl"]
      steer_cmd, throttle_cmd = map_actuators_to_pwm(cc.actuators, steer_max_deg, accel_gain)
      racecar.steering = steer_cmd
      racecar.throttle = throttle_cmd

      # carOutput 用に実際に出力した値を記録
      last_steer_deg = cc.actuators.steeringAngleDeg
      last_accel = cc.actuators.accel

    # pandaStates を毎ループ送る
    pm.send("pandaStates", make_panda_states_msg())

    # carState を擬似送信（定速・ゼロ舵角）
    pm.send("carState", make_car_state_msg(v_ego_stub, 0.0))

    # carOutput を毎ループ送る（controlsd が steer_limited_by_safety を判定するのに必要）
    pm.send("carOutput", make_car_output_msg(last_steer_deg, last_accel))

    # 擬似センサーデータを送信（selfdrived の sensorDataInvalid チェック回避）
    pm.send("accelerometer", make_sensor_msg("accelerometer"))
    pm.send("gyroscope", make_sensor_msg("gyroscope"))

    # carParams はたまに再送（controlsd 初期化漏れを防ぐため）
    now = time.monotonic()
    if now - last_cp_pub > 5.0:
      publish_car_params_once(pm, params, cp_reader, cp_bytes)
      last_cp_pub = now

    rk.keep_time()


if __name__ == "__main__":
  main()
