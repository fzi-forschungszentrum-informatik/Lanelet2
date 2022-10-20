#!/bin/bash
set -ex
SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
LANELET2_ROOT=$(dirname $SCRIPT_DIR)

export BUILD_DIR=/tmp/lanelet2_build_lint_$(id -u)

# setup mrt_cmake_modules, otherwise assume it's installed
set +x
source /opt/ros/*/setup.bash
set -x
if [[ -e $LANELET2_ROOT/../mrt_cmake_modules ]]; then
    mkdir -p $BUILD_DIR/mrt_cmake_modules
    pushd $BUILD_DIR/mrt_cmake_modules
    cmake $LANELET2_ROOT/../mrt_cmake_modules -DCMAKE_INSTALL_PREFIX=$BUILD_DIR/mrt_cmake_modules
    make install
    popd
fi

# make sure cmake finds everything
export CMAKE_PREFIX_PATH=$BUILD_DIR/mrt_cmake_modules:${CMAKE_PREFIX_PATH}
export AMENT_PREFIX_PATH=${LANELET2_ROOT}

# generate a compile_commands.json by configuring all lanelet2 packages in a big project
CMAKE_ROOT=${LANELET2_ROOT}/CMakeLists.txt
echo "cmake_minimum_required(VERSION 3.12)" > ${CMAKE_ROOT}
echo "project(lanelet2)" >> ${CMAKE_ROOT}
export LANELET2_PACKAGES_TOPOLOGICAL="lanelet2_core lanelet2_io lanelet2_projection lanelet2_traffic_rules lanelet2_routing lanelet2_maps lanelet2_validation lanelet2_matching lanelet2_python lanelet2_examples"
for pkg in $LANELET2_PACKAGES_TOPOLOGICAL; do
  echo "add_subdirectory($pkg)" >> ${CMAKE_ROOT};
  echo "set(${pkg}_LIBRARIES $pkg)" >> ${CMAKE_ROOT};
done
pushd $BUILD_DIR
cmake $LANELET2_ROOT -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DCATKIN_ENABLE_TESTING=1
popd

# run clang-tidy
find . -name "*.cpp" -print0 | xargs -0 -I{} -P$(nproc --all) clang-tidy-11 -p $BUILD_DIR {}