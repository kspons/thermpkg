# thermpkg

## Overview

This is a ROS package that publishes ThermApp camera raw data to ROS. The code is based off encryptededdy's ThermAppCam (https://github.com/encryptededdy/ThermAppCam/tree/master/thermapp)

**Keywords:** ROS, package, ThermApp

## Installation

Copy these files into a properly organized catkin_workspace. Use catkin_make to build the node. Source YOUR_CATKIN_WORKSPACE/devel/setup.bash

## Usage

First, run
  sudo modprobe v4l2loopback

Run the ROS node with
 	rosrun thermpkg thermpkg_node

Ros will have a "/temperature" node that contains the raw temperature data from the ThermApp camera. 
