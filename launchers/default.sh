#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# launching app
roscore&
sleep 5
dt-exec echo "Hallo"

ls packages/
cp -a packages/maps/. /usr/local/lib/python3.8/dist-packages/duckietown_world/data/gd1/maps

pip3 uninstall -y dataclasses

dt-exec Xvfb :1 -screen 0 1024x768x24 -ac +extension GLX +render -noreset
export DISPLAY=:1
#dt-exec rosrun my_package Test.py 
#dt-exec rosrun my_package Segment_debug.py
dt-exec roslaunch my_package wrapper.launch

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
