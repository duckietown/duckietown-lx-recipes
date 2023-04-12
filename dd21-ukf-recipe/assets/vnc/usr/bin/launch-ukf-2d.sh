#!/bin/bash -i

source /root/.bashrc
source /home/duckie/catkin_ws/devel/setup.bash
source /root/catkin_ws/devel/setup.bash
roslaunch --wait rosbridge_server rosbridge_websocket.launch&
python state_estimator.py --student --primary ukf2d --others simulator --ir_var $IR_VARIANCE_ESTIMATE