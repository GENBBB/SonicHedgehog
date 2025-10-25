#!/bin/bash

docker run -it --privileged \
    --network=host \
    --pid=host \
    -v /dev:/dev \
    -v .:/ros2_ws \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --env="DISPLAY" \
    sonic