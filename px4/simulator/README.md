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

Gimbal can be positioned through the px4 commander see https://dev.px4.io/en/advanced/gimbal_control.html
This will be necessary until the simulated gimbal tracking is functional

# Error troubleshooting
Errors about being unable to launch control_node.py or pose_estimation_node.py when running `./sim.sh`:
cd to the catkin_ws/src/drone/src/pose_estimation and `chmod +x control_node.py` and `chmod +x pose_estimation_node.py` 
These commands give roslaunch the permission to launch these nodes from the simulator.launch file when we run `./ros.sh`

Errors like  [Err] [Plugin.hh:165] Failed to load plugin
when running ./sim.sh if you do not specify the environment variable directory which sets the plugin path then ros cant find it.
If this is your first time running the animation plugin then you will need to open a new terminal outside of gazebo and createa a build directory for C. 
First cd to your AutoDock-sim/px4/simulator/scripts/gazebo/animatev7.
Run the following commands:
`mkdir build`
`cd build`
`cmake ../`
`make`
That creates a build directory and compiles the C plugins. Now to set the plugin directory to the build directory in the docker container go back to docker and cd to the build directory you just created and run
`export GAZEBO_PLUGIN_PATH=`pwd`:$GAZEBO_PLUGIN_PATH`
You will need to run this every time you run a new instance of `docker-compose up -d`

Error running `docker-compose up -d` such as 
ERROR: Version in "./docker-compose.yml" is unsupported. You might be seeing this error because you're using the wrong Compose file version. Either specify a version of "2" (or "2.0")
To solve this replace the first line in docker-compose.yml `version: "3.0"` with `version: "2.0"`
