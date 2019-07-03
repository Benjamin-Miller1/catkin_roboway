#!/bin/bash

if [ $# -lt 2 ]; then
    echo "error.. need name of carId and map"
    exit 1
fi

source /opt/ros/kinetic/setup.bash
source ~/workspace/catkin_roboway/devel/setup.bash

ROMOTEROSDIR="192.168.8.100:/home/roboway/catkin_roboway/src/bringup"

roscd bringup/map/$1
scp -q $2.pgm $2.yaml $2_modify.pgm $2_modify.yaml $2_path.json $ROMOTEROSDIR/map/$1