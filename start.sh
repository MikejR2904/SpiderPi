#!/bin/bash

# Function to handle termination signals
cleanup() {
    echo "Shutting down..."
    kill -SIGTERM "$roscore_pid"
    wait "$roscore_pid"
    kill -SIGTERM "$roslaunch_pid"
    wait "$roslaunch_pid"
    kill -SIGTERM "$mqtt_pid"
    wait "$mqtt_pid"
}

# Trap termination signals
trap cleanup SIGINT SIGTERM

# Source ROS environment
source /opt/ros/melodic/setup.bash
source /root/ws/devel/setup.bash

# Start ROS core in the background
roscore &
roscore_pid=$!
sleep 5

# Start RPLidar launch file in the background
roslaunch rplidar_ros rplidar_a1.launch &
roslaunch_pid=$!
sleep 5

# Start MQTT publisher in the background
python3 /root/ws/src/mqtt_publisher.py &
mqtt_pid=$!

# Wait for all background processes to finish
wait "$roscore_pid"
wait "$roslaunch_pid"
wait "$mqtt_pid"