#!/bin/bash

docker run -d --privileged \
  --network=host \
  --pid=host \
  -v /dev:/dev \
  -v .:/ros2_ws \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $HOME/.Xauthority:/root/.Xauthority:rw \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  --name sonic \
  rbegishev/sonic:sonic sleep infinity
