#!/bin/bash
# Franka Panda Color Sorter - ROS 2 Humble Docker Demo

xhost +local:docker
docker run -it --rm --name panda_demo \
  --network host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  curiousutkarsh/franka_panda_color_sorter:humble bash
