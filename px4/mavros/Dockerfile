FROM px4io/px4-dev-ros:latest

WORKDIR /src/firmware
RUN git clone https://github.com/PX4/Firmware . \
    && git submodule update --init --recursive
RUN make posix_sitl_default

# Start ROS
CMD ["bash", "-c", "source /opt/ros/kinetic/setup.bash && roslaunch mavros px4.launch fcu_url:=\"udp://:14540@127.0.0.1:14557\""]