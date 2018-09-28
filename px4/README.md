# PX4 Dev Environment Setup

https://dev.px4.io/en/simulation/gazebo.html

### 1.) Install Docker

https://tuleap-documentation.readthedocs.io/en/latest/developer-guide/quick-start/install-docker.html

### 2.) Download Graphics Drivers

Download drivers from https://www.nvidia.com/object/Unix.html

Note: make sure to match the current driver version: `nvidia-smi`

Put the *.run file in the /simulator-nvidia directory and rename it to NVIDIA-DRIVER.run

TODO Link to guide

### 3.) Build simulator image

In this directory, run `docker-compose build`

This will automatically pull docker images and setup your environment locally.

If you are on BSU's network, there's a good chance the container will fail to build. 
It would appear that BSU blocks most public DNS servers (8.8.8.8, 1.1.1.1, etc), so
you have to make sure you're using the default.

Refer to this post to point Docker at the correct DNS server:
https://stackoverflow.com/a/40516974


# Running the simulator

Run the following command outside of docker. This will give docker access to the display.

`xhost +`

To start the container, run `docker-compose up -d simulator`

To connect to the simulator, run the `./connect` script.

Once inside the container, you can cd to `/home/user/scripts` and
run `./sim.sh` to start the simulator.