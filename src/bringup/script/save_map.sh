#!/bin/bash
if [ $# -lt 1 ]; then
    echo "error.. need a name for map file"
    exit 1
fi

source /opt/ros/kinetic/setup.bash
source ~/workspace/catkin_roboway/devel/setup.bash

roscd bringup/map
rosrun map_server map_saver -f $1
cp $1.pgm $1_modify.pgm
cp $1.yaml $1_modify.yaml
sed -i s/$1/$1_modify/ $1_modify.yaml

roscd bringup/script
python modify_launch.py $1

