# PX4 Dev Environment Setup

1.) Install docker

2.) Install docker-compose

3.) Download NVidia drivers

Download drivers from https://www.nvidia.com/object/Unix.html
Put the *.run file in this directory and rename it to NVIDIA-DRIVER.run

TODO Link to guide

4.) Build the simulator `docker-compose build simulator`

This will automatically pull docker images and setup your environment locally.

If you are on BSU's network, there's a good chance the container will fail to build. 
It would appear that BSU blocks most public DNS servers (8.8.8.8, 1.1.1.1, etc), so
you have to make sure you're using the default.

Refer to this question to point Docker at the correct DNS server:
https://stackoverflow.com/a/40516974


# Running the simulator

Run the following command outside of docker. This will give docker access to the display.

`xhost +`

To start the container, run `docker-compose up -d`

To connect to the container, run the `./connect` script.

Once inside the container, you can cd to `/home/user/AutoDock/px4/scripts` and
run `./sim.sh` to start the simulator.

To start QGroundControl, run `./qgroundcontrol.sh`