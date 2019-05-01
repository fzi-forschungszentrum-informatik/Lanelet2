#!/usr/bin/env bash

# this file needs to be executed with "source setup.sh"

source /root/.bashrc
source /opt/ros/kinetic/setup.bash


echo "1. Building catkin repo(s)"

mkdir -p $(pwd)/build
mkdir -p $(pwd)/logs
mkdir -p $(pwd)/devel

ln -s $(pwd)/lanelet2_modules $(pwd)/src

catkin init
catkin build

echo "2. Setting environment variables"
source devel/setup.bash
