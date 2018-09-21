#!/usr/bin/env bash

cd /src/firmware
make posix_sitl_default
make posix_sitl_default sitl_gazebo
make posix gazebo_typhoon_h480