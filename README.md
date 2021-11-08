## begineer_tutorials
Tutorial on how to create a simple service to chnage the message

## Overview
* This repo shows how to write a simple publisher and subscriber and then to use a service to modify the message in C++11
* The version of ROS used is melodic
* This tutorial is created by referencing http://wiki.ros.org/ROS/Tutorials
* The launch file "week10.launch" starts a publisher and suscriber. After
  8 seconds  the client sents a request to the server to change the message in the publisher
* The user can also change frquency of publisher during launch

## rqt_console screenshot
![image](https://github.com/markosej11/begineer_tutorials/blob/Week10_HW/Screenshot.png

## Requirements
* catkin
* ROS melodic
* Ubuntu 18.04

## Build Instructions
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ git clone --recursive https://github.com/markosej11/begineer_tutorials/tree/main
$ source devel/setup.bash
$ cd ..
$ catkin_make
$ source ~/catkin_ws/devel/setup.bash
```

## Running Instructions
```
roslaunch begineer_tutorials week10.launch 
```
The above will launch the 2 nodes and the talker will publish at 8hz default. 
```
roslaunch begineer_tutorials week10.launch pub_freq:=<Enter an integer value here>
```
The above will launch the 2 nodes and the talker will publish at the input frequency chosen by you.


 
