# Lanelet2

[![CI](https://github.com/fzi-forschungszentrum-informatik/lanelet2/actions/workflows/ci.yaml/badge.svg)](
https://github.com/fzi-forschungszentrum-informatik/lanelet2/actions/workflows/ci.yaml)
[![CD](https://github.com/fzi-forschungszentrum-informatik/lanelet2/actions/workflows/cd.yaml/badge.svg)](
https://github.com/fzi-forschungszentrum-informatik/lanelet2/actions/workflows/cd.yaml)
[![Build Status ROS focal/noetic](https://build.ros.org/job/Ndev__lanelet2__ubuntu_focal_amd64/lastBuild/badge/icon?subject=ROS%20noetic%20%28on%20focal%29)](
https://build.ros.org/job/Ndev__lanelet2__ubuntu_focal_amd64/lastBuild/)
[![Build Status ROS2 focal/foxy](https://build.ros2.org/job/Fdev__lanelet2__ubuntu_focal_amd64/lastBuild/badge/icon?subject=ROS2%20foxy%20%28on%20focal%29)](
https://build.ros2.org/job/Fdev__lanelet2__ubuntu_focal_amd64/lastBuild/)
[![Build Status ROS2 jammy/humble](https://build.ros2.org/job/Hdev__lanelet2__ubuntu_jammy_amd64/lastBuild/badge/icon?subject=ROS2%20humble%20%28on%20jammy%29)](
https://build.ros2.org/job/Hdev__lanelet2__ubuntu_jammy_amd64/lastBuild/)
[![PyPI Downloads](https://img.shields.io/pypi/dm/lanelet2.svg?label=PyPI%20downloads)](
https://pypi.org/project/lanelet2/)

## Overview

Lanelet2 is a C++ library for handling map data in the context of automated driving. It is designed to utilize high-definition map data in order to efficiently handle the challenges posed to a vehicle in complex traffic scenarios. Flexibility and extensibility are some of the core principles to handle the upcoming challenges of future maps.

Features:
- **2D and 3D** support
- **Consistent modification**: if one point is modified, all owning objects see the change
- Supports **lane changes**, routing through areas, etc.
- **Separated routing** for pedestrians, vehicles, bikes, etc.
- Many **customization points** to add new traffic rules, routing costs, parsers, etc.
- **Simple convenience functions** for common tasks when handling maps
- **Accurate Projection** between the lat/lon geographic world and local metric coordinates
- **IO Interface** for reading and writing e.g. _osm_ data formats (this does not mean it can deal with _osm maps_)
- **Python** bindings for the whole C++ interface
- **Boost Geometry** support for all thinkable kinds of geometry calculations on map primitives
- Released under the [**BSD 3-Clause license**](LICENSE)
- Support for **ROS1, ROS2, Docker and Conan** (see instructions below)

![](lanelet2_core/doc/images/lanelet2_example_image.png)

Lanelet2 is the successor of the old [liblanelet](https://github.com/phbender/liblanelet/tree/master/libLanelet) that was developed in 2013. If you know Lanelet1, you might be interested in [reading this](lanelet2_core/doc/Lanelet1Compability.md).

## Documentation

You can find more documentation in the individual packages and in doxygen comments. Here is an overview on the most important topics:
- [Here](lanelet2_core/doc/LaneletPrimitives.md) is more information on the basic primitives that make up a Lanelet2 map.
- [Read here](lanelet2_core/doc/Architecture.md) for a primer on the **software architecture** of lanelet2.
- There is also some [documentation](lanelet2_core/doc/GeometryPrimer.md) on the geometry calculations you can do with lanelet2 primitives.
- If you are interested in Lanelet2's **projections**, you will find more [here](lanelet2_projection/doc/Map_Projections_Coordinate_Systems.md).
- To get more information on how to create valid maps, see [here](lanelet2_maps/README.md).

## Installation

### Within ROS
Lanelet2 has been released for ROS. Just install `ros-[distribution]-lanelet2`, e.g.:
```
sudo apt install ros-noetic-lanelet2
```

### Without ROS
Outside of ROS, Lanelet2 can be installed from PyPI. Note that currently only Python 3.8-3.11 linux builds are available and that Python 3.10+ is only supported for recent linux distributions such as Ubuntu 20.04+.
```
pip install lanelet2
```
#### Note:

If you receive the error

```
ERROR: Could not find a version that satisfies the requirement lanelet2 (from versions: none)
ERROR: No matching distribution found for lanelet2
```

during installation, even when using e.g. python 3.9 or 3.8 on a somewhat recent linux such as Ubuntu 18.04 or newer, your pip version is probably too old, 
as e.g. the pip version that comes with apt on Ubuntu 20.04 (20.0.2) is not recent enough for the provided package.

In this case you need to simply update pip with 

```
pip3 install -U pip 
```

### Using Docker

There is a Docker container from which you can test things out:

```
docker build -t lanelet2 .                    # builds a docker image named "lanelet2"
docker run -it --rm lanelet2:latest /bin/bash # starts the docker image
python -c "import lanelet2"                   # quick check to see everything is fine
```

The docker image contains a link to your local lanelet2, so you can work and see changes (almost) at the same time. Work with two screens, one local and one on docker. Make your code changes locally, then run again `catkin build` on docker to recompile the code (update python modules).

### Manual installation

In case you want to build it in your own way (without the above Docker image) use these instructions.

Lanelet2 relies mainly on [Catkin](https://catkin-tools.readthedocs.io/en/latest/index.html) for building and is targeted towards Linux.

At least **C++14** is required.

### Dependencies
Besides [Catkin](https://catkin-tools.readthedocs.io/en/latest/index.html), the dependencies are
* `Boost` (from 1.58)
* `eigen3`
* [`mrt_cmake_modules`](https://github.com/KIT-MRT/mrt_cmake_modules), a CMake helper library
* `pugixml` (for lanelet2_io)
* `boost-python, python2 or python3` (for lanelet2_python)
* `geographiclib` (for lanelet2_projection)
* `rosbash` (for lanelet2_examples)

For Ubuntu, the steps are the following:
* [Set up ROS](http://wiki.ros.org/ROS/Installation), and install at least `rospack`, `catkin` and `mrt_cmake_modules` (e.g. `ros-melodic-rospack`, `ros-melodic-catkin`, `ros-melodic-mrt-cmake-modules`):
```
sudo apt-get install ros-melodic-rospack ros-melodic-catkin ros-melodic-mrt-cmake-modules
```

* Install the dependencies above:
```bash
sudo apt-get install libboost-dev libeigen3-dev libgeographic-dev libpugixml-dev libpython-dev libboost-python-dev python-catkin-tools
```

**On 16.04 and below**, `mrt_cmake_modules` is not available in ROS and you have to clone it into your workspace (`git clone https://github.com/KIT-MRT/mrt_cmake_modules.git`).

### Building
As usual with Catkin, after you have sourced the ros installation, you have to create a workspace and clone all required packages there. Then you can build.
```shell
source /opt/ros/$ROS_DISTRO/setup.bash
mkdir catkin_ws && cd catkin_ws && mkdir src
catkin init
catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo # build in release mode (or whatever you prefer)
cd src
git clone https://github.com/fzi-forschungszentrum-informatik/lanelet2.git
cd ..
catkin build
```

If unsure, see the [Dockerfile](Dockerfile) or the [travis build log](https://travis-ci.org/fzi-forschungszentrum-informatik/Lanelet2). It shows the full installation process, with subsequent build and test based on a docker image with a clean Ubuntu installation.

### Manual, experimental installation using conan
For non-catkin users, we also offer a conan based install process. Its experimental and might not work on all platforms, especially Windows.
Since conan handles installing all C++ dependencies, all you need is a cloned repository, conan itself and a few python dependencies:
```bash
pip install conan catkin_pkg numpy
conan remote add bincrafters https://bincrafters.jfrog.io/artifactory/api/conan/public-conan # required for python bindings
conan config set general.revisions_enabled=1 # requried to use bincrafters remote
git clone https://github.com/fzi-forschungszentrum-informatik/lanelet2.git
cd lanelet2
```

From here, just use the default conan build/install procedure, e.g.:
```bash
conan source .
conan create . lanelet2/stable --build=missing
```
Different from the conan defaults, we build lanelet2 and boost as shared libraries, because otherwise the lanelet2's plugin mechanisms as well as boost::python will fail. E.g. loading maps will not be possible and the python API will not be usable.

To be able to use the python bindings, you have to make conan export the PYTHONPATH for lanelet2:
```bash
conan install lanelet2/0.0.0@lanelet2/stable --build=missing -g virtualenv # replace 0.0.0 with the version shown by conan
source activate.sh
python -c "import lanelet2" # or whatever you want to do
source deactivate.sh
```

### Python3

The python bindings are build for your default python installation by default (which currently is python2 on most systems). To build for python3 instead of python2, create a python3 virtualenv before initializing the workspace with `catkin init`. The command `python` should point to `python3`.

After `catkin init` run `catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DPYTHON_VERSION=3.6` to make sure that the correct python version is used. Then build and use as usual.

*Note: With bionic and beyond, the apt package `python3-catkin-tools` conflicts with ROS melodic and should not be used. Use either the python2 version or use pip to install the python3 version.*

## Examples
Examples and common use cases in both C++ and Python can be found [here](lanelet2_examples/README.md).

## Packages
* **lanelet2** is the meta-package for the whole lanelet2 framework
* **lanelet2_core** implements the basic library with all the primitives, geometry calculations and the LanletMap object
* **lanelet2_io** is responsible for reading and writing lanelet maps
* **lanelet2_traffic_rules** provides support to interpret the traffic rules encoded in a map
* **lanelet2_projection** for projecting maps from WGS84 (lat/lon) to local metric coordinates
* **lanelet2_routing** implements the routing graph for routing or reachable set or queries as well as collision checking
* **lanelet2_maps** provides example maps and functionality to visualize and modify them easily in JOSM
* **lanelet2_matching** provides functions to determine in which lanelet an object is/could be currently located
* **lanelet2_python** implements the python interface for lanelet2
* **lanelet2_validation** provides checks to ensure a valid lanelet2 map
* **lanelet2_examples** contains tutorials for working with Lanelet2 in C++ and Python

## Citation

If you are using Lanelet2 for scientific research, we would be pleased if you would cite our [publication](http://www.mrt.kit.edu/z/publ/download/2018/Poggenhans2018Lanelet2.pdf):
```latex
@inproceedings{poggenhans2018lanelet2,
  title     = {Lanelet2: A High-Definition Map Framework for the Future of Automated Driving},
  author    = {Poggenhans, Fabian and Pauls, Jan-Hendrik and Janosovits, Johannes and Orf, Stefan and Naumann, Maximilian and Kuhnt, Florian and Mayr, Matthias},
  booktitle = {Proc.\ IEEE Intell.\ Trans.\ Syst.\ Conf.},
  year      = {2018},
  address   = {Hawaii, USA},
  owner     = {poggenhans},
  month     = {November},
  Url={http://www.mrt.kit.edu/z/publ/download/2018/Poggenhans2018Lanelet2.pdf}
}
```

