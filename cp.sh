export ROAD_CAM='nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=30/1 ! nvvidconv flip-method=0 ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! appsink max-buffers=1 drop=true sync=false'
# export ROAD_CAM='nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=640, height=360, format=NV12, framerate=20/1 ! queue max-size-buffers=1 leaky=downstream ! nvvidconv flip-method=0 ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! appsink max-buffers=1 drop=true sync=false' \

# Stop leftovers from previous runs to avoid double-start issues.
PIDS=$(pgrep -f "system/manager/manager.py|tools.webcam.camerad|selfdrive.modeld.modeld" || true)
if [ -n "$PIDS" ]; then
  echo "killing old processes: $PIDS"
  kill $PIDS || true
  sleep 1
  PIDS=$(pgrep -f "system/manager/manager.py|tools.webcam.camerad|selfdrive.modeld.modeld" || true)
  if [ -n "$PIDS" ]; then
    echo "force killing old processes: $PIDS"
    kill -9 $PIDS || true
  fi
fi

export DISPLAY=:0
export XAUTHORITY=/run/user/1000/gdm/Xauthority
export RCD_RACECAR_TYPE=RAW
export DEV=CUDA
export DEVICE=CUDA
export RCD_I2C_BUS=7
export RCD_STEER_MAX_DEG=25.0
export RCD_RAW_PARAMS=/home/jetson/jetracer/notebooks/raw_params.json
export USE_WEBCAM=1
export USE_FAKE_PANDA=1
export DISABLE_DMON=1
export ROAD_CAM="$ROAD_CAM"
./system/manager/manager.py
