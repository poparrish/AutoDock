FROM px4io/px4-dev-ros@sha256:03ba39eaae4eaeac2d332470375173f2bca65f7b80e9c0e7966c1254158f3410

## Build OpenCV w/ GStreamer
# https://www.pyimagesearch.com/2015/06/22/install-opencv-3-0-and-python-2-7-on-ubuntu/
WORKDIR /src/opencv
RUN git clone https://github.com/opencv/opencv.git .
RUN git checkout 3.3.1

RUN apt-get install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev

WORKDIR /src/opencv/build
RUN cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D WITH_GSTREAMER=ON -D BUILD_opencv_python2=ON ../
RUN make -j8
RUN make install && ldconfig

## Clone and build PX4 firmware
WORKDIR /src/firmware
RUN git clone https://github.com/PX4/Firmware .
RUN git checkout v1.8.0 && git submodule update --init --recursive
RUN sed -i 's/"enable gstreamer plugin" "OFF"/"enable gstreamer plugin" "ON"/g' "Tools/sitl_gazebo/CMakeLists.txt"
RUN make posix_sitl_default
RUN make posix_sitl_default sitl_gazebo

## Install virtual camera runtime dependencies
RUN apt-get update
RUN apt-get install -y libudev-dev libsdl2-dev gstreamer1.0-* libgstreamer1.0-*

## Install Nvidia drivers
# Download drivers from https://www.nvidia.com/object/Unix.html
# Make sure to match your current driver version (view by running nvidia-smi)
# Put the *.run file in this directory and rename it to NVIDIA-DRIVER.run
RUN apt-get install kmod -y
COPY NVIDIA-DRIVER.run .
RUN ./NVIDIA-DRIVER.run -a -N --ui=none --no-kernel-module && rm ./NVIDIA-DRIVER.run

## Setup ROS
# This is a hack to use our custom build of OpenCV
RUN rm /opt/ros/kinetic/lib/python2.7/dist-packages/cv2.so

WORKDIR /root/catkin_ws/
RUN bash -c "source /opt/ros/kinetic/setup.bash && \
             mkdir src && \
             catkin_make" && \
    echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

# Build plugins
WORKDIR /root/plugins/
COPY scripts/gazebo/animatev7 .
RUN mkdir build && cd build && cmake .. && make && \
    echo "source /root/plugins/setup.bash" >> /root/.bashrc

WORKDIR /root/scripts

## Keep the container alive
CMD ["bash", "-c", "tail -f /dev/null"]
