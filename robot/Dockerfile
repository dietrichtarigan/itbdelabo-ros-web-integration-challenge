FROM ros:noetic-desktop-full

# ROS packages for TurtleBot3 sim, rosbridge, and camera streaming
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-turtlebot3 \
    ros-noetic-turtlebot3-simulations \
    ros-noetic-rosbridge-server \
    ros-noetic-web-video-server \
    python3-flask \
    && rm -rf /var/lib/apt/lists/*

ENV TURTLEBOT3_MODEL=waffle
WORKDIR /root

# Launch file (starts Gazebo + rosbridge + web_video_server)
COPY sim_stack.launch /root/sim_stack.launch

# Minimal REST control service to start/stop the stack on demand
# (keeps container light: file created during build)
RUN printf '%s\n' \
    "from flask import Flask, jsonify, request" \
    "from subprocess import Popen" \
    "import os, signal" \
    "app = Flask(__name__)" \
    "proc = None" \
    "@app.route('/start', methods=['POST','GET'])" \
    "def start():" \
    "  global proc" \
    "  if proc and proc.poll() is None:" \
    "    return jsonify(status='already_running')" \
    "  proc = Popen(['roslaunch','--wait','/root/sim_stack.launch'])" \
    "  return jsonify(status='starting')" \
    "@app.route('/stop', methods=['POST','GET'])" \
    "def stop():" \
    "  global proc" \
    "  if proc and proc.poll() is None:" \
    "    proc.terminate()" \
    "    try: proc.wait(timeout=10)" \
    "    except Exception: proc.kill()" \
    "  os.system('pkill -f gzserver || true'); os.system('pkill -f rosbridge_websocket || true'); os.system('pkill -f web_video_server || true')" \
    "  return jsonify(status='stopped')" \
    "@app.route('/health')" \
    "def health():" \
    "  running = bool(proc and proc.poll() is None)" \
    "  return jsonify(running=running)" \
    "if __name__ == '__main__':" \
    "  app.run(host='0.0.0.0', port=5000)" > /root/control.py

EXPOSE 9090 8080 5000
CMD ["bash", "-lc", "python3 /root/control.py"]
