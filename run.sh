set -e

source /opt/ros/noetic/setup.bash

roslaunch foxglove_bridge foxglove_bridge.launch &
sleep 2

python3 subscriber/main.py