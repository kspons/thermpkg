# Thermpkg

## Overview

This package allows the ThermApp camera to be used in ROS. It usues encryptededdy's ThermAppCam and umlaeute's v4l2loopback

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [v4l2loopback](https://github.com/umlaeute/v4l2loopback) (camera access for Ubuntu),
- [ThermAppCam](https://github.com/encryptededdy/ThermAppCam/tree/master/thermapp) (ThermApp camera as webcam)

#### Building

Clone this repository into your catkin workspace and compile using catkin_make

    cd catkin_workspace/src
	git clone https://github.com/kspons/thermpkg.git
	cd ../
	catkin_make

