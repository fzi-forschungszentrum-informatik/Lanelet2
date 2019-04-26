#!/bin/bash

echo "Building catkin repo(s)"
source /root/.bashrc
source /opt/ros/kinetic/setup.bash

catkin init
catkin build

source devel/setup.bash

# this may not work if all the "source" commands aren't executed! 
pip install scripts/
