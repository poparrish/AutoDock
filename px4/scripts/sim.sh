#!/usr/bin/env bash

firmware="/src/firmware"

# Make sure virtual camera is enabled (requires rebuild)
sed -i 's/"enable gstreamer plugin" "OFF"/"enable gstreamer plugin" "ON"/g' "$firmware/Tools/sitl_gazebo/CMakeLists.txt"

# Copy custom world files
cp -rf ../gazebo/* "$firmware/Tools/sitl_gazebo"

# Build & launch simulation
cd "$firmware"
if [[ $* == *--clean* ]]
then
    make clean
fi
make posix gazebo_typhoon_h480