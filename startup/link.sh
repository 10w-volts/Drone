#!/bin/bash
sleep 15
sudo chmod 777 /dev/ttyTHS0
#python gpio_init.py

source /opt/ros/melodic/setup.bash
source /home/nvidia/Code/cartographer_ws/install_isolated/setup.bash
source /home/nvidia/Code/catkin_ws/devel/setup.bash
source /home/nvidia/Code/mavros_ws/devel/setup.bash

#export ROS_MASTER_URI=http://192.168.1.138:11311
#export ROS_HOSTNAME=192.168.1.138

#sleep 3
#roslaunch whud_union whud_union.launch
