FROM px4io/px4-dev-ros:latest

## Build OpenCV w/ GStreamer
# https://www.pyimagesearch.com/2015/06/22/install-opencv-3-0-and-python-2-7-on-ubuntu/
WORKDIR /src/opencv
RUN git clone https://github.com/opencv/opencv.git .
RUN git checkout 3.3.1

RUN apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev -y

WORKDIR /src/opencv/build
RUN cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D WITH_GSTREAMER=ON -D BUILD_opencv_python2=ON ../
RUN make -j8
RUN make install && ldconfig

## Build PX4 firmware
WORKDIR /src/firmware
RUN git clone https://github.com/PX4/Firmware .
RUN git checkout v1.8.0 && git submodule update --init --recursive
# TODO Less hacky way to enable video streaming. Mabye mount firmware in its own volume?
RUN sed -i 's/"enable gstreamer plugin" "OFF"/"enable gstreamer plugin" "ON"/g' Tools/sitl_gazebo/CMakeLists.txt \
    && sed -i 's/<!--<gui>/<gui>/g' Tools/sitl_gazebo/worlds/typhoon_h480.world \
    && sed -i 's/<\/gui>-->/<\/gui>/g' Tools/sitl_gazebo/worlds/typhoon_h480.world
RUN make posix_sitl_default
RUN make posix_sitl_default sitl_gazebo

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
# This is a hack to use our custom build of OpenCV
RUN rm /opt/ros/kinetic/lib/python2.7/dist-packages/cv2.so
RUN echo "source /opt/ros/kinetic/setup.bash" >> /root/.bashrc

WORKDIR /home/user

## Keep the container alive
CMD ["bash", "-c", "tail -f /dev/null"]