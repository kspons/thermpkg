# Thermpkg

## Overview

This package allows the ThermApp camera to be used in ROS. It usues encryptededdy's ThermAppCam and umlaeute's v4l2loopback. I've primarily used and developed this package to use the ThermApp camera on UAVs. Using Ubuntumate on a raspberrypi, I am able to capture aerial thermal images on a 3DR Solo.

**Keywords:** ROS, ThermApp, thermal

**Author: Keith Sponsler

Affiliation: Texas A&M University**

Thermpkg has been tested under [ROS] Kinetic using Ubuntu 17.04 and Ubuntumate for Raspberry Pi Model B. I oftentimes make and break code on here, especially the python scripts. But the main publisher should remain intact. 

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

Ensure the python scripts are executable

    cd thermpkg/scripts
    chmod +x *.py

## Usage

Before you can use the ThermApp camera with ROS, you must have v4l2loopback running

    sudo modprobe v4l2loopback

The main node that you'll want to run is the thermpkg_node. It will publish the temperature data

    rosrun thermpkg thermpkg_node

To view the image data as a grey image, use tempView

    rosrun thermpkg tempView.py

## Nodes

### Published Topics
* **'/temperature'** ([std_msgs/Int16MultiArray])
    The temperature matrix from the ThermApp camera.



## Bugs & Feature Requests

Please report bugs and request features using github, or send me, Keith Sponsler, an email.