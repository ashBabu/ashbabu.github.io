---
layout: post
title: "Part1: Developing a ROS C++ package"
date: 2021-11-06 11:30:00-0400
description: How to make a ROS package
categories: Robotics Programming
---
### Introduction

This introduces the various stages of developing a simple Robot Operating System (ROS) package in Linux. The main aim is to focus on some good practices that are required to make a ROS compatible package. Open up the terminal and type the following

* `mkdir -p ~/catkin_ws/src`    ## This will simply create two folders `src` inside `catkin_ws`
* `cd ~/catkin_ws/src`
* `catkin_create_pkg my_test_package roscpp std_msgs moveit_core`  

The `catkin_create_pkg` has to be understood careflly. The first argument is the name of the package (`my_test_package`) and the rest are dependencies [roscpp](http://wiki.ros.org/roscpp), [std_msgs](http://wiki.ros.org/std_msgs) and [moveit](https://moveit.ros.org/). Each package would have its own dependencies and could be different. After executing the above three lines, the `src` folder would have two other important files viz., `CMakeLists.txt` and [package.xml](http://wiki.ros.org/catkin/package.xml). For the time being, lets leave both these as it is but it would be interesting to take a look at both these files and how ROS has organized our dependencies (`roscpp, std_msgs, moveit_core`) in it. The final step is to do the following

* `cd ~/catkin_ws`
* `catkin_make`

The `catkin_make` is equivalent to 

* `cd ~/catkin_ws`
* `mkdir build`
* `cd build`
* `cmake ../src -DCMAKE_INSTALL_PREFIX=../install -DCATKIN_DEVEL_PREFIX=../devel`
* `make`

At this stage the folder structure would be
```
catkin_ws
    |_ build/
    |_ deve/l
    |_ src/
        |_ CMakeLists.txt
        |_ my_test_pkg/
                |_ CMakeLists.txt
                |_ package.xml
                |_ src/
                |_ include/
                      |_ my_test_pkg/

```
We will organize and add our c++ header files in `include/my_test_package` and all c++ source files in `src`
