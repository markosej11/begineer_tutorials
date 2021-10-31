# begineer_tutorials
Tutorial on how to create a simple publisher and subscriber on C++11 on ROS melodic

# Overview
* This repo shows how to write a simple publisher and subscriber in C++11
* The version of ROS used is melodic
* This tutorial is created by referencing http://wiki.ros.org/ROS/Tutorials

# Requirements
* catkin
* ROS melodic
* Ubuntu 18.04

# Build Instructions
'''
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone --recursive https://github.com/markosej11/begineer_tutorials/tree/main
source devel/setup.bash
cd ..
catkin_make
source ~/catkin_ws/devel/setup.bash
'''

# Running Instructions
terminal 1
'''
cd ~/catkin_ws/src/begineer_tutorials
source ~/catkin_ws/devel/setup.bash
roscore
'''

terminal 2
'''
cd ~/catkin_ws/src/begineer_tutorials
source ~/catkin_ws/devel/setup.bash
rosrun beginner_tutorials talker
'''

terminal 3
'''
cd ~/catkin_ws/src/begineer_tutorials
source ~/catkin_ws/devel/setup.bash
rosrun beginner_tutorials listener
'''



 
