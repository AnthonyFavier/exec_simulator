#!/usr/bin/env bash
cd ../src

git clone https://github.com/ros-controls/urdf_geometry_parser.git && \
git clone https://github.com/pal-robotics/ddynamic_reconfigure.git && \
git clone https://github.com/DLu/wu_ros_tools.git && \
git clone https://github.com/wg-perception/people.git
# # to be added to src folder (checkout "noetic" branch for people pkg)
cd people && git checkout noetic

cd ../..
git clone https://github.com/AnthonyFavier/HATPEHDA.git
cd HATPEHDA && git checkout concurrent

cd ../docker

image_name=hatpehda_noetic
docker build --build-arg UID=$(id -u) --build-arg GID=$(id -g) -t ${image_name} $(dirname "$0")/