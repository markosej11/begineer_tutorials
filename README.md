## begineer_tutorials
Tutorial on how to use TF broadcaster and rosbags

## Overview
* This repo shows how to write a simple publisher and subscriber and then to use a service to modify the message in C++11
* It also contains the demonstration of a broadcaster which uses TF library as well as the use of ROS bags and gtesting
* The version of ROS used is melodic
* This tutorial is created by referencing http://wiki.ros.org/ROS/Tutorials
* The launch file "week10.launch" starts a publisher and suscriber. After
  8 seconds  the client sents a request to the server to change the message in the publisher
* The user can also change frquency of publisher during launch

## rqt_console screenshot
![image](https://github.com/markosej11/begineer_tutorials/blob/Week10_HW/Screenshot.png)

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
roslaunch begineer_tutorials week11.launch 
```
The above will launch the 2 nodes and the talker will publish at 8hz default. 
```
roslaunch begineer_tutorials week11.launch pub_freq:=<Enter an integer value here>
```
The above will launch the 2 nodes and the talker will publish at the input frequency chosen by you.

The above commands also starts a rosbag which records for 15 seconds.


## To inspect TF frames
First terminal
```
roslaunch begineer_tutorials week11.launch
```
Second Terminal
```
rosrun tf view_frames
```
This will create a PDF with all the frames in the current directory the terminal is on
To view the PDF run the below command from the second terminal
```
evince frames.pdf
```

## To visualize the tree of frames being broadcasted
First terminal
```
roslaunch begineer_tutorials week11.launch
```
Second Terminal
```
rosrun rqt_tf_tree rqt_tf_tree
``` 

## To see the transform between the world frame and talk frame
First terminal
```
roslaunch begineer_tutorials week11.launch
```
Second Terminal
```
rosrun tf tf_echo world talk
``` 

## To run rostest
To run the unit tests for th ros service run the below command
```
cd ~/catkin_make
catkin_make run_tests_begineer_tutorials
```

## To record bag files
simply launching the nodes using the launch file will start recoding in rosbags for 15 sec
```
roslaunch begineer_tutorials week11.launch
```

To disable the bag file recording simply do the following
```
roslaunch begineer_tutorials week11.launch ifRecord:=false
```

## To inspect the bag file
first launch the nodes using the below command
```
roslaunch begineer_tutorials week11.launch
```
Then close the nodes and start roscore using the below command
```
roscore
```
On a second terminal navigate to the directory which contains the bag file
```
cd ~/catkin_ws/src/begineer_tutorials/results/bags
```
To inspect the file run the below command
```
rosbag info sample.bag 
```

## To use the bag file with listener node
First terminal
```
roscore
```
Second terminal
```
rosrun begineer_tutorials listener
```
Third terminal
```
cd ~/catkin_ws/src/begineer_tutorials/results/bags
rosbag play sample.bag
```