#!/usr/bin/env bash

swd="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

mkdir -p $swd/build
mkdir -p $swd/logs
mkdir -p $swd/devel

echo "Building catkin repo(s)"
source /root/.bashrc
source /opt/ros/kinetic/setup.sh


ln -s $swd/lanelet2_modules $swd/src

catkin init
catkin build

source $swd/devel/setup.sh

