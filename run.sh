#!/usr/bin/env bash

until docker ps > /dev/null
do
    echo "Waiting for docker server"
    sleep 1
done

swd="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

echo "Starting container lanelet2..."

docker run \
      -it \
      --rm \
      --net host \
      -v $swd:/home/workspace/lanelet2_modules \
      -v $swd/setup.sh:/home/workspace/setup.sh \
      lanelet2:latest

