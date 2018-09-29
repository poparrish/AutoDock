#!/usr/bin/env bash

cd /root/AutoDock/px4/firmware

# Enable virtual camera
sed -i 's/"enable gstreamer plugin" "OFF"/"enable gstreamer plugin" "ON"/g' Tools/sitl_gazebo/CMakeLists.txt
sed -i 's/<!--<gui>/<gui>/g' Tools/sitl_gazebo/worlds/typhoon_h480.world
sed -i 's/<\/gui>-->/<\/gui>/g' Tools/sitl_gazebo/worlds/typhoon_h480.world

if [[ $* == *--clean* ]]
then
    make clean
fi
make posix gazebo_typhoon_h480