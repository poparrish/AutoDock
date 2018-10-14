#!/usr/bin/env bash

firmware="/src/firmware"

# Copy custom world files
cp -rf ./gazebo/* "$firmware/Tools/sitl_gazebo"

# Build & launch simulation
cd "$firmware"
if [[ $* == *--clean* ]]
then
    make clean
fi
make posix gazebo_typhoon_h480