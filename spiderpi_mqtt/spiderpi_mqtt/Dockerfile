# Use an appropriate base image
FROM ros:melodic-ros-base

# Install necessary dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    ros-melodic-rplidar-ros && \
    pip3 install paho-mqtt pyyaml rospkg rosdep

# Set up the workspace
RUN mkdir -p /root/ws/src && \
    cd /root/ws/src && \
    git clone https://github.com/Slamtec/rplidar_ros.git && \
    cd /root/ws && \
    /bin/bash -c "source /opt/ros/melodic/setup.bash && catkin_make"

# Source the ROS setup file
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc && \
    echo "source /root/ws/devel/setup.bash" >> ~/.bashrc

# Copy your MQTT node or script
COPY ./mqtt_publisher.py /root/ws/src/mqtt_publisher.py

# Copy the start script
COPY ./start.sh /root/start.sh
RUN chmod +x /root/start.sh

# Define the command to run rplidar and MQTT publisher
CMD ["/root/start.sh"]
