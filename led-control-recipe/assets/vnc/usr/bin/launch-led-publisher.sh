#!/bin/bash
echo "Launching the LED publisher ..."
rostopic pub --latch "/${VEHICLE_NAME}/led_node/update" std_msgs/Bool "data: true" &
python3 /root/Documents/tk-window.py