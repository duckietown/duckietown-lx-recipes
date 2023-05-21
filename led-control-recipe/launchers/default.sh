#!/bin/bash

source /environment.sh
source /code/submission_ws/devel/setup.bash --extend
source /code/solution/devel/setup.bash --extend

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------

# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# launching app

dt-exec roslaunch --wait led_publisher led_publisher_node.launch veh:=$VEHICLE_NAME

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join