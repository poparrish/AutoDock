#!/usr/bin/env bash

# Start QGroundControl (run xhost + first)
# For whatever reason, this program cannot be run as root
runuser -l user -c "cd ~/qgroundcontrol; ./qgroundcontrol-start.sh"