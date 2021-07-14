# FROM gdwyer/ros:foxy
FROM ros:foxy

# Install any needed packages then clean up apt cache
RUN apt-get update && apt-get install -y \
    ssh \
    ros-foxy-rqt \
    python3-pip \
    vim \
 && rm -rf /var/lib/apt/lists/*

RUN pip3 install gpiozero pigpio

# Copies perfusion system stack into image
RUN mkdir -p /dev_ws/src/ros_pi_daq
COPY . /dev_ws/src/ros_pi_daq/

# Build the workspace so you're ready to go
WORKDIR /dev_ws
# RUN ["/bin/bash", "-c", "source /ros2_foxy/install/setup.bash &&\
#      colcon build"]
RUN ["/bin/bash", "-c", "source /opt/ros/foxy/setup.bash &&\
     colcon build"]

RUN cp /dev_ws/src/ros_pi_daq/.docker/entrypoint.sh /
RUN chmod 777 /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]

CMD ["ros2 run daq_server daq_server"]
