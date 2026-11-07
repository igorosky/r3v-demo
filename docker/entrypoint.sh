#!/bin/bash
# With GUI=1, starts an X display shared through noVNC on 6080. Then runs the command.
set -e
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash

if [ "${GUI:-0}" = 1 ] || [ "${GUI:-0}" = true ]; then
  export DISPLAY=:1
  Xvfb :1 -screen 0 1920x1080x24 -nolisten tcp &
  sleep 1
  openbox &
  x11vnc -display :1 -forever -shared -nopw -quiet -rfbport 5900 &
  websockify --web /usr/share/novnc 6080 localhost:5900 > /dev/null 2>&1 &
fi

exec "$@"
