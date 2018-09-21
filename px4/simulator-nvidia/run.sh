#!/usr/bin/env bash

#source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/posix_sitl_default
#export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/sitl_gazebo

cd /src/firmware
source /opt/ros/kinetic/setup.bash
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/posix_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch px4 mavros_posix_sitl.launch

#roslaunch px4 multi_uav_mavros_sitl.launch
#roslaunch px4 posix_sitl.launch
#ls /home/user/.ros/log/
#ls /home/user/.ros/log/latest/
#roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
#cat /home/user/.ros/log/latest/*.log

#roslaunch gazebo_ros empty_world.launch paused:=true use_sim_time:=false gui:=true throttled:=false recording:=false debug:=true verbose:=true
#roslaunch px4 posix_sitl.launch
#make posix_sitl_default gazebo
#
#ls ./log/
tail -f /dev/null