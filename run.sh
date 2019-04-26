#!/usr/bin/env bash

until docker ps > /dev/null
do
    echo "Waiting for docker server"
    sleep 1
done


echo "Starting container lanelet2..."

mkdir -p src
mkdir -p build
mkdir -p devel
mkdir -p logs

docker run \
      -it \
      --rm \
      --net host \
      -v $(pwd)/lanelet2_modules:/home/workspace/src/lanelet2 \
      -v $(pwd)/mrt_cmake_modules:/home/workspace/src/mrt_cmake_modules \
       -v $(pwd):/home/workspace \
      --entrypoint bash \
      lanelet2

