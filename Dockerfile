from ubuntu:16.04

# basics 
RUN apt-get update && apt-get install -y \
        build-essential \
        cmake \
        wget \
        vim \
        nano \
        gdb \
        valgrind \
        git \
        python-dev \
        python-pip \
        lsb-core \
        && apt-get clean && rm -rf /var/lib/apt/lists/*

# install ROS (melodic)
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116 \
    && apt-get update \
    && apt-get install -y ros-kinetic-ros-base \
    && apt-get clean && rm -rf /car/lib/apt/lists/*

# dependencies for lanelet2
RUN apt-get update && apt-get install -y \
        libboost-dev \
        libeigen3-dev \
        libgeographic-dev \
        libpugixml-dev \
        libpython-dev \
        libboost-python-dev \
        python-catkin-tools \
        && apt-get clean && rm -rf /var/lib/apt/lists/*


# environment and entry points
ENV HOME /home/workspace
WORKDIR /home/workspace

RUN mkdir /home/workspace/src \
    && git clone https://github.com/KIT-MRT/mrt_cmake_modules.git \
        /home/workspace/src/mrt_cmake_modules
