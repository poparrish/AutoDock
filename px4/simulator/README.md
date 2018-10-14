# PX4 Dev Environment Setup

https://dev.px4.io/en/simulation/gazebo.html

### 1.) Install Docker

https://tuleap-documentation.readthedocs.io/en/latest/developer-guide/quick-start/install-docker.html

### 2.) Download Graphics Drivers (Nvidia)

If you don't have an Nvidia card or don't use the proprietary drivers, you can 
comment out the driver installation commands in the simulator Dockerfile.

Download drivers from https://www.nvidia.com/object/Unix.html

Note: make sure to match the current driver version, which you can obtain by 
running the `nvidia-smi` command

Put the *.run file in the ./simulator directory and rename it to NVIDIA-DRIVER.run

If you encounter problems with your graphics drivers, refer to this guide:
https://dev.px4.io/en/test_and_ci/docker.html#graphics-driver-issues
http://gernotklingler.com/blog/howto-get-hardware-accelerated-opengl-support-docker/

### 3.) Build simulator image

In this directory, run `docker-compose build`

# Running the simulator

Run the following command outside of docker. This will give docker access to the display.

`xhost +local:root`

To start the px4 container, run `docker-compose up -d`

Once the container is running, connect by running `./connect.sh`

After connecting, run `./sim.sh` to start Gazebo and `./ros.sh` to start mavros