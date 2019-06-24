ARG DISTRIBUTION=18.04
FROM ubuntu:${DISTRIBUTION} AS lanelet2_deps

ARG ROS_DISTRO=melodic

# basics
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
	    bash-completion \
        build-essential \
	    curl \
        git \
        cmake \
        ipython \
	    keyboard-configuration \
	    locales \
        lsb-core \
        nano \
        python-dev \
        software-properties-common \
        sudo \
	    wget \
	    && locale-gen en_US.UTF-8 \
        && apt-get clean && rm -rf /var/lib/apt/lists/*

# locale
ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US:en \
    LC_ALL=en_US.UTF-8 \
    ROS_DISTRO=${ROS_DISTRO}

# install ROS
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list \
    && (apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
      || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654)

# dependencies for lanelet2
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
        libboost-all-dev \
        libeigen3-dev \
        libgeographic-dev \
        libpugixml-dev \
        libpython-dev \
        libboost-python-dev \
        python-catkin-tools \
        ros-$ROS_DISTRO-catkin \
        ros-$ROS_DISTRO-rosbash \
        ros-$ROS_DISTRO-ros-environment # missing dep of rospack on xenial \
        && apt-get clean && rm -rf /var/lib/apt/lists/*

# create a user
RUN useradd --create-home --groups sudo --shell /bin/bash developer \
    && mkdir -p /etc/sudoers.d \
    && echo "developer ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/developer \
    && chmod 0440 /etc/sudoers.d/developer


# environment, dependencies and entry points
USER developer
ENV HOME /home/developer
WORKDIR /home/developer/workspace

RUN sudo chown -R developer:developer /home/developer \
    && echo "export ROS_HOSTNAME=localhost" >> /home/developer/.bashrc \
    && echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/developer/.bashrc \
    && echo "source /home/developer/workspace/devel/setup.bash" >> /home/developer/.bashrc

# setup workspace, add dependencies
RUN cd /home/developer/workspace \
    && /bin/bash -c "source /home/developer/.bashrc && catkin init && catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo" \
    && git clone https://github.com/KIT-MRT/mrt_cmake_modules.git /home/developer/workspace/src/mrt_cmake_modules

# second stage: get the code and build the image
FROM lanelet2_deps As lanelet2

# bring in the code
COPY --chown=developer:developer . /home/developer/workspace/src/lanelet2

# update dependencies
RUN git -C /home/developer/workspace/src/mrt_cmake_modules pull

# build
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && catkin build --no-status"

