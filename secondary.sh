#!/bin/bash

export LANG=en_US.UTF-8
LOGFILE=/home/pi/secondary_startup.log

{
    # Wait for a short period to ensure that the initial startup script has completed
    sleep 10

    echo "Starting pigpio daemon..." >> $LOGFILE
    sudo pigpiod

    echo "Killing existing instances of the servers..." >> $LOGFILE
    sudo pkill -f SpiderPi.py

    sudo kill -9 $(sudo lsof -t -i:8080)
    sudo kill -9 $(sudo lsof -t -i:9030)

    # Navigate to the directory where your SpiderPi scripts are located
    cd /home/pi/SpiderPi

    # Start SpiderPi.py
    echo "Starting SpiderPi..." >> $LOGFILE
    python3 SpiderPi.py

    # Run SpiderPi.py again to ensure it is up and running
    sleep 5
    echo "Restarting SpiderPi..." >> $LOGFILE
    sudo pkill -f SpiderPi.py
    python3 SpiderPi.py
} >> $LOGFILE 2>&1