export ROAD_CAM='nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=30/1 ! nvvidconv flip-method=0 ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! appsink max-buffers=1 drop=true sync=false'
# export ROAD_CAM='nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=640, height=360, format=NV12, framerate=20/1 ! queue max-size-buffers=1 leaky=downstream ! nvvidconv flip-method=0 ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! appsink max-buffers=1 drop=true sync=false' \

DISPLAY=:0 \
XAUTHORITY=/run/user/1000/gdm/Xauthority \
RCD_RACECAR_TYPE=RAW \
DEV=CUDA \
DEVICE=CUDA \
RCD_I2C_BUS=7 \
RCD_STEER_MAX_DEG=25.0 \
RCD_RAW_PARAMS=/home/jetson/jetracer/notebooks/raw_params.json \
USE_WEBCAM=1 \
USE_FAKE_PANDA=1 \
DISABLE_DMON=1 \
ROAD_CAM="$ROAD_CAM" \
BLOCK=loggerd,encoderd \
./system/manager/manager.py
