---
layout: post
title: "Custom Inverse Kinematics MoveIt Plugin"
date: 2022-06-16 11:30:00-0400
description: How to make a Inverse Kinematics Plugin Compatable with MoveIt
categories: Robotics Programming
---
### Introduction

Solving inverse kinematics (IK) is a fundamental problem in robotics that converts task space co-ordinates to joint space variables. To solve for joint space values, `q's`, we require non-linear transcendental equations connecting the task-space and joint-space values or forward kinematic equations. As a simple case to understand this, lets take the case of a single degree of freedom planar robot arm having a revolute joint. We can write the forward kinematics as
\begin{align}
    x = l * \cos(q) \nonumber \newline
    y = l * \sin(q) \nonumber
\end{align}
where `l` and `q` are respectively the link length and the angle that the link makes with the fixed co-ordinate system. The inverse kinematics (given (x,y), what is the value of `q`?) can be written as 

\begin{align}
    \tan (q) = \frac{y}{x} \implies q = \arctan(\frac{y}{x})
    \label{eqn:pend_ik}
\end{align}

Equation \ref{eqn:pend_ik} is a very simple example of an analytic inverse kinematics solution. [IKFast](http://openrave.org/docs/0.8.2/openravepy/ikfast/) is one such analytic IK solver for robot arms having any number of joints and is quite popular. However, it's often difficult to set it up and there are a lot of "IKFast Not Working" queries if you do a google search. 

In [ROS](https://www.ros.org/), the most popular way of setting up a robot is using [URDF](http://wiki.ros.org/urdf/Tutorials) and I will discuss another simple case of generating symbolic forward kinematics equations using [sympy](https://www.sympy.org/en/index.html) in [another post](/blog/2022/Generating_Symbolic_Expression_for_Forward_Kinematics/). For now, I will assume that we already have an analytic expression for the IK.

This article will discuss the various steps involved in creating a custom IK plugin to be used along with [MoveIt](https://moveit.ros.org/). (Why MoveIt because it is a very powerful and popular open source library for motion planning problems and is used by several robots). Firstly, we need to create a plugin using [pluginlib](http://wiki.ros.org/pluginlib), [pluginlib_tutorials](http://wiki.ros.org/pluginlib/Tutorials/Writing%20and%20Using%20a%20Simple%20Plugin). The KDL and SRV kinematics plugin [documentation](https://moveit.ros.org/documentation/plugins/#kinematicsbase) give a very good idea of how to get started and set it up. I am just writing my experiences here for a quick overview of various things involved. So the steps involved are as follows
* Open a terminal and `cd` to `catkin_ws/src` and 
```
catkin_create_pkg moveit_kinematics_plugin roscpp rospy pluginlib moveit_core
```
* `cd moveit_kinematics_plugin/include/moveit_kinematics_plugin` and create a `TestKinematicsPlugin.h`([here](https://github.com/ashBabu/moveit_kinematics_plugin/blob/master/include/moveit_kinematics_plugin/TestKinematicsPlugin.h)). 
* Most importatnly, this header file should inherit the `kinematics::KinematicsBase` and implement the pure virtual methods `getPositionFK()`, `getJointNames()`, `getLinkNames()`, the overloaded `getPositionIK()` and the `searchPositionIk()`. This also should have the `initialize()` method for initialization using the new syntax which takes the `robot_model` as the first argument instead of the deprecated `robot_description`.
* I have added three other private functions, `getIKSolutions()`, `isSolutionValid()` and `getNearestSolutionToSeed()` for my specific case. 
* [getIKSolutions()](https://github.com/ashBabu/moveit_kinematics_plugin/blob/master/src/TestKinematicsPlugin.cpp#L188) implements the solution (for example equation \ref{eqn:pend_ik}). Something **Important** that I found here is that if you have two joint variables `q1` and `q2` which respectively being the first and second joints, the `solutions` vector should implement something like  `solutions.push_back({q2, q1})`. (Something to be investigated again)
* `isSolutionValid()` checks if the solution found is within the joint limits and `getNearestSolutionToSeed()` finds the solution that is closest to the seed state provided. This is implemented for my specific robot and may not be required in general.
* The last but very important bit is the implementation of one of the overloaded `searchPositionIK()` method. I have numbered it as 4. Go through it and copy paste the function as such
* The `solution_callback()` is provided if the configuration of the robot arm corresponding to the solution is collision free or not. This should be implemented for additional safety but the program would work even without this.
* Add the following as well in the `TestKinematicsPlugin.cpp`
```
#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(moveit_kinematics_plugin::TestKinematicsPlugin, kinematics::KinematicsBase);
```
* Create a `plugin_description.xml` file in the same level as `src` that should look like 

```
<?xml version='1.0' encoding='UTF-8'?>
<library path="lib/test_arm_moveit_plugin">
  <class base_class_type="kinematics::KinematicsBase" name="analytic/kinematicsPlugin" type="moveit_kinematics_plugin::TestArmKinematicsPlugin">
    <description>MoveIt plugin for solving the kinematics analytically </description>
  </class>
</library>
```
* Add an `export` tag in the `package.xml` and it should look like
```
  <export>
    <moveit_core plugin="${prefix}/plugin_description.xml"/>
  </export>
```
* Copy the missing contents of the `CMakeLists.txt` into your `CMakeLists.txt` especially the `install` parts
* `cd` to your `catkin_ws` and do `catkin_make` and `source devel/setup.bash`.
* When it finishes, you should see a `test_arm_moveit_plugin.so` (check the `plugin_description.xml` library path) generated under `catkin_ws/devel/lib`. It's often in my case, the plugin is not regenerated if an IDE like CLion is used (dont know why!!). So make sure to check the time of generation of the `test_arm_moveit_plugin.so` by typing `ll devel/lib | grep test_arm`. This will give the time of the latest generation of the plugin. Or as a last resort, remove `build` and `devel` folders and rerun `catkin_make` if the plugin is not generated.
* Additionally run `rospack plugins --attrib=plugin moveit_core` on your console. This should list the full path of `plugin_description.xml`. 
* Modify the `robot_moveit_config/config/kinematics.yaml` with the following
```
arm:
    kinematics_solver: analytic/kinematicsPlugin
```
**Note:** The `robot_moveit_config` package is generated using [MoveIt Setup Assistant](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html?highlight=setup%20assistant). 

Congratulations, Your plugin is ready to be launched when you start your simulation