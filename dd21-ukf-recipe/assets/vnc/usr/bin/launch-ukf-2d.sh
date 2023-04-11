#!/bin/bash
source /home/duckie/catkin_ws/devel/setup.bash
roslaunch --wait rosbridge_server rosbridge_websocket.launch&
python /home/duckie/catkin_ws/src/pidrone_pkg/scripts/state_estimator.py --primary ukf2d --others simulator --ir_var $IR_VARIANCE_ESTIMATE --student