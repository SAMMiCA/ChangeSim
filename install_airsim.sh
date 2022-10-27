#!/bin/bash

git submodule update --init --recursive
cd external/AirSim
./setup.sh
./build.sh
cd PythonClient
pip install .
cd ..
sudo apt-get install ros-noetic-tf2-sensor-msgs ros-noetic-mavros* python3-catkin-tools ros-noetic-ros-numpy

cd ros
catkin_make
cd ../../..
cp script/utils/settings.json ~/Documents/AirSim/settings.json

