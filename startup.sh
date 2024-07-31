#!/bin/bash

export LANG=en_US.UTF-8

LOGFILE=/home/pi/startup.log

{
# Check and start the pigpio daemon if not already running
sudo pigpiod

# Kill any existing instances of the servers
echo "Killing existing instances of the servers..." >> $LOGFILE
sudo pkill -f RPCServer.py
sudo pkill -f Camera.py
sudo pkill -f SpiderPi.py
sudo pkill -f MjpgServer.py

# Grant access to serial port
sudo chmod a+rw /dev/ttyUSB0

# Free ports 8080 and 9030 if occupied
echo "Freeing ports 8080 and 9030 if occupied..." >> $LOGFILE
sudo kill 9 $(sudo lsof -t -i:8080)
sudo kill 9 $(sudo lsof -t -i:9030)

# Navigate to the directory where your SpiderPi scripts are located
cd /home/pi/SpiderPi

# Start RPCServer.py
echo "Starting RPCServer..."
python3 RPCServer.py &

# Start Camera.py
echo "Starting Camera..."
python3 Camera.py &

# Start SpiderPi.py
echo "Starting SpiderPi..."
python3 SpiderPi.py &

# Start MjpgServer.py
echo "Starting MjpgServer..."
python3 MjpgServer.py &

cd /home/pi

echo "Restarting Secondary Startup...."
/home/pi/secondary.sh &

cd /home/pi/SpiderPi/HiwonderSDK

# Start IMU MQTT Publisher
echo "Starting to publish IMU data..."
python3 Mpu6050.py &

# Start Docker
sudo systemctl start docker

sleep 10

# Function to build Docker images if not already built
build_docker_images() {
    # Check if the rplidar_mqtt image exists
    if [[ "$(docker images -q rplidar_mqtt 2> /dev/null)" == "" ]]; then
        echo "Building rplidar_mqtt Docker image..."
        cd /home/pi/docker_ros
        sudo docker build -t rplidar_mqtt .
    fi
}

# Function to run Docker containers if not already running
run_docker_containers() {
    # Run RPLidar container if not already running
    if [ ! "$(docker ps -q -f name=rplidar_container)" ]; then
        if [ "$(docker ps -aq -f status=exited -f name=rplidar_container)" ]; then
            # Cleanup
            sudo docker stop rplidar_container
            sudo docker rm rplidar_container
        fi
        echo "Running rplidar_container..."
        sudo docker run -d --restart unless-stopped --name rplidar_container --device=/dev/ttyUSB0 -p 8883:1883 rplidar_mqtt
    fi
}

# Build Docker images if needed
build_docker_images

# Run Docker containers
run_docker_containers

echo "All services started."
} >> $LOGFILE 2>&1