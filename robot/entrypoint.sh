#!/usr/bin/env bash
set -euo pipefail

ROSBRIDGE_PORT="${ROSBRIDGE_PORT:-9090}"
VIDEO_PORT="${VIDEO_PORT:-8080}"
SIM_MODE="${SIM_MODE:-fake}"   # jangan nyalain Gazebo

log(){ echo "[$(date +'%F %T')] $*"; }

cleanup(){
  pkill -f turtlebot3_fake_node || true
  pkill -f robot_state_publisher || true
  pkill -f image_publisher || true
  pkill -f rosbridge_websocket || true
  pkill -f web_video_server || true
  pkill -f roscore || true
}
trap cleanup EXIT

log "===== Source ROS env ====="
source /opt/ros/noetic/setup.bash
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-waffle}"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
export ROS_IP="${ROS_IP:-127.0.0.1}"

log "===== Start roscore ====="
roscore & sleep 2

# tunggu master siap
for i in {1..30}; do
  if rosparam list >/dev/null 2>&1; then
    log "[OK] roscore is up"
    break
  fi
  sleep 0.5
done

if ! rosparam list >/dev/null 2>&1; then
  log "[ERR] roscore failed"
  exit 1
fi

if [ "$SIM_MODE" = "fake" ]; then
  log "===== Start turtlebot3_fake_node ====="
  rosrun turtlebot3_fake turtlebot3_fake_node >/tmp/tb3_fake.log 2>&1 & disown
  sleep 1

  # (opsional) state publisher – boleh ada, gak wajib
  rosrun robot_state_publisher robot_state_publisher >/tmp/rsp.log 2>&1 & disown || true

  # ===== Smart Camera Source =====
  if [ -f /assets/demo.mp4 ]; then
    log "[*] Using video_stream_opencv -> /camera/image_raw (from /assets/demo.mp4)"
    rosrun video_stream_opencv camera_node _video_stream_provider:=/assets/demo.mp4 \
      _fps:=15 _topic:=/camera/image_raw >/tmp/cam.log 2>&1 & disown
  else
    log "[*] Using static image_publisher -> /camera/image_raw"
    IMG="/opt/ros/noetic/share/rviz/images/splash.png"
    rosrun image_publisher image_publisher "$IMG" __name:=camera >/tmp/cam.log 2>&1 & disown
  fi
  sleep 1

  # verifikasi topik kamera muncul
  for i in {1..30}; do
    if rostopic list | grep -q '^/camera/image_raw$'; then
      log "[OK] camera topic: /camera/image_raw"
      break
    fi
    sleep 0.5
  done
  if ! rostopic list | grep -q '^/camera/image_raw$'; then
    log "[ERR] camera topic tidak muncul. Dump log:"
    tail -n +1 /tmp/cam.log || true
  fi
fi

log "===== Start rosbridge on :${ROSBRIDGE_PORT} ====="
pkill -f rosbridge_websocket || true
roslaunch rosbridge_server rosbridge_websocket.launch address:=0.0.0.0 port:=${ROSBRIDGE_PORT} \
  >/tmp/rosbridge.log 2>&1 & disown
for i in {1..30}; do
  if nc -z 127.0.0.1 ${ROSBRIDGE_PORT}; then
    log "[OK] rosbridge ws://0.0.0.0:${ROSBRIDGE_PORT}"
    break
  fi
  sleep 0.5
done

log "===== Start web_video_server on :${VIDEO_PORT} ====="
pkill -f web_video_server || true
rosrun web_video_server web_video_server _port:=${VIDEO_PORT} \
  >/tmp/webvideo.log 2>&1 & disown
for i in {1..30}; do
  if nc -z 127.0.0.1 ${VIDEO_PORT}; then
    log "[OK] web_video_server http://0.0.0.0:${VIDEO_PORT}"
    break
  fi
  sleep 0.5
done

log "===== Robot stack RUNNING ====="
# debug ringkas
log "Topics:"
rostopic list || true

tail -f /dev/null
