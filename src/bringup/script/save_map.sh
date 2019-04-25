#!/bin/bash
if [ $# -lt 1 ]; then
    echo "error.. need a name for map file"
    exit 1
fi

source ~/code/catkin_roboway/devel/setup.bash
source ~/code/catkin_qt/devel/setup.bash
export ROS_MASTER_URI=http://192.168.8.100:11311
export ROS_IP=192.168.8.101

roscd bringup/map
#rosrun map_server map_saver -f $1
cp $1.pgm $1_modify.pgm
cp $1.yaml $1_modify.yaml
sed -i s/$1/$1_modify/ $1_modify.yaml

roscd bringup/script
python modify_launch.py $1

