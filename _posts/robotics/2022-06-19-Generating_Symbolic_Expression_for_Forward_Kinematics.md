---
layout: post
title: "Generating Symbolic Expression for Forward Kinematics"
date: 2022-06-19 11:30:00-0400
description: How to generate symbolic expression for forward kinematics
categories: Robotics Programming
---

The method that is going to be discussed here applies to simple manipulators. A more general form could be derived but the aim here is to understand the basics. The way [ROS](https://www.ros.org/) understands the kind of robot that is being loaded into the [parameter server](http://wiki.ros.org/Parameter%20Server) is through `robot_description` tag which is basically an `xml` file more commonly known as [URDF](http://wiki.ros.org/urdf/Tutorials). The URDF file could be parsed to extract all the relevant parameters to compute the forward kinematics (Refer [orocos kdl](https://www.orocos.org/kdl.html) for deeper understanding). 

Let's take the case of a simple three degree of freedom planar manipulator. Refer to this [python script](https://github.com/ashBabu/moveit_kinematics_plugin/blob/master/scripts/forward_kinematics.py) for the following discussion. Some of the things are hard coded as I am not doing any URDF parsing. Launch the simulation environment so that URDF is loaded into the parameter server. Send the robot to zero position (where all the joint angles are zero).Then we need to know the link names as  described in the URDF (`self.links`). The `arm_base` to `arm_link_0` and `arm_link_3` to `arm_end_effector` are transformations where the rotations are constant (usually identity) and some fixed translation. Since we are considering a 3-DOF planar manipulator, we know that there are three joint space variable viz.,`q1`, `q2` and `q3`. The following table can be obtained by carefully observing the output produced by the `get_trans_rot()` function. It takes in a source and target frame and returns the translation, quaternion and the 4 x 4 transformation matrix between them. 


|                                           |   Rotation<br>(Axis, Angle)   |       Translation         |
|------------------------------------------ |-----------------------------  |-------------------------- |
| arm_base to arm_link0                     | Identity                      | [0.02, 0.0, -0.0901]      |
| arm_link0 to arm_link1                    | (Z, q1)                       | [0.1215, 0.0, -0.004]     |
| arm_link1 to arm_link2                    | (Z, -q2)                      | [0.2, 0.0, 0.004]         |
| arm_link2 to arm_link3                    | (Z, q3)                       | [0.2, 0.0, 0.0]           |
| arm_link3 to arm_end_effector             | Identity                      | [0.24, 0.0, 0.045]        |

The symbolic forward kinematics is computed using [sympy](https://www.sympy.org/en/index.html) and is referenced under the name `self.tfMat_abEff`. There is a helper function `forward_kinematics()` which simplifies the expression and returns the `x`, `y` and `z` co-ordinates and the orientation of the end-effector with respect to the arm base ($$\phi$$). For the particular case of my robot, I got the equations as
\begin{align}
     x &= 0.2\cos(q1) + 0.2\cos(q1 - q2) + 0.25\cos(q1 - q2 + q3) + 0.1415 \nonumber \newline
     y &= 0.2\sin(q1) + 0.2\sin(q1 - q2) + 0.25\sin(q1 - q2 + q3)  \nonumber
\end{align}