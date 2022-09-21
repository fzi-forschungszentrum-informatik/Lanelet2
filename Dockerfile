# The ubuntu distro to use
ARG DISTRIBUTION=20.04
FROM ubuntu:${DISTRIBUTION} AS lanelet2_deps

# Select the ros distro
ARG ROS_DISTRO=noetic

# The ros version of that distro (either ros or ros2)
ARG ROS=ros

# If true, build docker container for development
ARG DEV=0

SHELL ["/bin/bash", "-c"]

# basics
RUN set -ex; \
    if [ "${ROS_DISTRO}" = "melodic" ] || [ "${ROS_DISTRO}" = "kinetic" ]; \
        then export PY_VERSION=python; \
        else export PY_VERSION=python3; \
    fi; \
    if [ "$DEV" -ne "0" ]; then \
        export DEV_PACKAGES="clang-format-11 clang-tidy-11 clang-11 i${PY_VERSION} nano lcov"; \
    fi; \
    apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
        bash-completion \
        build-essential \
        curl \
        git \
        cmake \
        keyboard-configuration \
        locales \
        lsb-core \
        lib${PY_VERSION}-dev \
        software-properties-common \
        sudo \
        wget \
        ${DEV_PACKAGES} && \
    locale-gen en_US.UTF-8 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# locale
ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US:en \
    LC_ALL=en_US.UTF-8 \
    ROS_DISTRO=${ROS_DISTRO} \
    ROS=${ROS} \
    DEV=${DEV}

# install ROS
RUN set -ex; \
    export KEY_FILE=/usr/share/keyrings/ros-archive-keyring.gpg && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o $KEY_FILE && \
    echo "deb [signed-by=$KEY_FILE] http://packages.ros.org/${ROS}/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros.list

# dependencies for lanelet2
RUN if [ "${ROS_DISTRO}" = "melodic" ] || [ "${ROS_DISTRO}" = "kinetic" ]; \
        then export PY_VERSION=python; \
        else export PY_VERSION=python3; \
    fi; \
    apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
        libgtest-dev \
        libboost-all-dev \
        libeigen3-dev \
        libgeographic-dev \
        libpugixml-dev \
        libboost-python-dev \
        ${PY_VERSION}-rospkg \
        ros-$ROS_DISTRO-ros-environment && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# ros version specific dependencies
RUN set -ex; \
    if [ "${ROS_DISTRO}" = "melodic" ] || [ "${ROS_DISTRO}" = "kinetic" ]; \
        then export PY_VERSION=python; \
        else export PY_VERSION=python3; \
    fi; \
    if [ "$ROS" = "ros" ]; \
        then export ROS_DEPS="ros-$ROS_DISTRO-catkin ros-$ROS_DISTRO-rosbash ${PY_VERSION}-catkin-tools"; \
        else export ROS_DEPS="ros-$ROS_DISTRO-ament-cmake python3-colcon-ros ros-$ROS_DISTRO-ros2cli"; \
    fi; \
    apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y $ROS_DEPS && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# create a user
RUN useradd --create-home --groups sudo --shell /bin/bash developer && \
    mkdir -p /etc/sudoers.d && \
    echo "developer ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/developer && \
    chmod 0440 /etc/sudoers.d/developer


# environment, dependencies and entry points
USER developer
ENV HOME /home/developer
WORKDIR /home/developer/workspace

RUN set -ex; \
    sudo chown -R developer:developer /home/developer && \
    if [ "$ROS" = "ros" ]; then export SPACE="devel"; else export SPACE="install"; fi; \
    echo "export ROS_HOSTNAME=localhost" > /home/developer/.bashrc && \
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/developer/.bashrc && \
    echo "source /home/developer/workspace/${SPACE}/setup.bash || true" >> /home/developer/.bashrc

# setup workspace, add dependencies
RUN set -ex; \
    cd /home/developer/workspace && \
    mkdir -p /home/developer/workspace/src && \
    if [ "$ROS" = "ros" ]; then \
      source /home/developer/.bashrc && \
      catkin init; \
    fi; \
    git clone https://github.com/KIT-MRT/mrt_cmake_modules.git /home/developer/workspace/src/mrt_cmake_modules

# second stage: get the code
FROM lanelet2_deps AS lanelet2_src

# bring in the code
COPY --chown=developer:developer . /home/developer/workspace/src/lanelet2

# update dependencies
RUN git -C /home/developer/workspace/src/mrt_cmake_modules pull

# third stage: build
FROM lanelet2_src AS lanelet2

# build
RUN set -ex; \
    if [ "$DEV" -ne "0" ]; then \
      export CMAKE_ARGS="-DCMAKE_BUILD_TYPE=Debug -DMRT_SANITIZER=checks -DMRT_ENABLE_COVERAGE=1"; \
    else \
      export CMAKE_ARGS="-DCMAKE_BUILD_TYPE=Release"; \
    fi; \
    if [ "$ROS" = "ros" ]; then \
      export CONFIGURE="catkin config --cmake-args $CMAKE_ARGS"; \
      export BUILD_CMD="catkin build --no-status"; \
    else \
      export CONFIGURE=true; \
      export BUILD_CMD="colcon build --symlink-install --cmake-args $CMAKE_ARGS"; \
    fi; \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    env && \
    $CONFIGURE && \
    $BUILD_CMD && \
    if [ "$DEV" -eq "0" ]; then \
      rm -rf build logs; \
    fi