FROM px4io/px4-dev-ros:latest

## Clone and build firmware
WORKDIR /src/firmware
RUN git clone https://github.com/PX4/Firmware . \
    && git submodule update --init --recursive
RUN make posix_sitl_default
RUN make posix_sitl_default sitl_gazebo

## Install QGroundControl
WORKDIR /home/user
RUN wget https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.tar.bz2
RUN tar jxf QGroundControl.tar.bz2 && rm QGroundControl.tar.bz2

## Install virtual camera dependencies
#RUN apt-get update
#RUN apt-get install -y \
#    gstreamer1.0-* \
#    libgstreamer1.0-* \
#    speech-dispatcher \
#    libudev-dev \
#    libsdl2-dev

## Install Nvidia drivers
# Download drivers from https://www.nvidia.com/object/Unix.html
# Put the *.run file in this directory and rename it to NVIDIA-DRIVER.run
RUN apt-get install kmod
COPY NVIDIA-DRIVER.run .
RUN ./NVIDIA-DRIVER.run -a -N --ui=none --no-kernel-module && rm ./NVIDIA-DRIVER.run

## Setup ROS
RUN echo "source /opt/ros/kinetic/setup.bash" >> /root/.bashrc

## Keep the container alive
CMD ["bash", "-c", "tail -f /dev/null"]