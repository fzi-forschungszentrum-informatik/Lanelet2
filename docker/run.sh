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
      -v $swd/../:/home/developer/workspace/lanelet2_modules \
      lanelet2:latest

