#!/bin/bash

docker run -d --privileged \
    --network=host \
    --pid=host \
    -v /dev:/dev \
    -v .:/ros2_ws \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --env="DISPLAY" \
    --name sonic \
    sonic sleep infinity