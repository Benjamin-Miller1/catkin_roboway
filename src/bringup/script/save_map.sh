#!/bin/bash
if [ $# -lt 2 ]; then
    echo "error.. need name of carID and map"
    exit 1
fi

source /opt/ros/kinetic/setup.bash
source ~/workspace/catkin_roboway/devel/setup.bash

roscd bringup/map
mkdir -p $1
cd $1
rosrun map_server map_saver -f $2
cp $2.pgm $2_modify.pgm
cp $2.yaml $2_modify.yaml
sed -i s/$2.pgm/$2_modify.pgm/ $2_modify.yaml
