#!/bin/bash

if [ $# -lt 1 ]; then
    echo "error.. need a name for map file"
    exit 1
fi

echo $ROS_PACKAGE_PATH

roscd bringup/launch

ROMOTEROSDIR="192.168.8.100:/home/autolabor/catkin_roboway/src/bringup"

scp -q amcl.launch move_base.launch $ROMOTEROSDIR/launch

roscd bringup/map
scp -q $1.pgm $1.yaml $1_modify.pgm $1_modify.yaml $1_path.json $ROMOTEROSDIR/map