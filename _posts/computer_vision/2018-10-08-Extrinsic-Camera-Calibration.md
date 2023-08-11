---
layout: post
title: Extrinsic Camera Calibration Using Optimization
date: 2018-10-08 09:15:00-0400
description: How to calibrate a camera with respect to robot's base
categories: Computer-Vision
---

## Theory 

<img style="float: left;" title="Camera calibration" src="/assets/img/cam_calib.png" alt="Camera Calibration" width="750" height="400"/>



Figure shows the schematic of the robot where `B`, `EE` and `C` represent the base frame (or fixed frame) of the robot, end-effector frame and camera frame respectively. `M` corresponds to the origin of the marker (april-tags) which is along the Z axis of the `EE` frame. We know the translation vector, $$^Bt_{EE} $$, from the `tf` topic from the robot. The attachment fitted with the `EE` is made in SolidWorks and hence the translation,$$^{EE}t_M$$ is also known. From the detection algorithm running, we get the translation, $$ ^Ct_M $$. From all these known information, we can write
 
 \begin{equation}
 ^Bt_{EE} + ^B[R]_{EE} ~^{EE}t_M  = ^Bt_C + ^B[R]_C ~^Ct_M
 \end{equation}
 
where $$ ^B[R]_{EE} $$ and $$ ^B[R]_C $$ are the 3 x 3 rotation matrices relating the base to end-effector and base to camera co-ordinate systems. The left hand side of the equation is fully known and it is required to find the unknowns in $$ ^Bt_C $$ and $$ ^B[R]_C $$. Since $$ ^B[R]_C $$ is a rotation matrix, it follows the following (orthogonal matrix)

    * the dot product of each of the columns should be zero
    * the sum of the squares of the elements in each column should be 1

Then formulate an constrained optimization problem which is to minimize the error 

\begin{equation}
   Min~ Z = \sum_{i=1}^{n} (X_i - ^Bt_C - ^B[R]_C ~^C{t_M}_i)^2
\end{equation}

For detailed info regarding how to formulate an constrained optimization is available [here](https://docs.scipy.org/doc/scipy/reference/optimize.html). Checkout the `SLSQP` method in particular.

## Other Resources
*. [Youtube: MoveIt - Easy Hand Eye Calibration with MoveIt](https://www.youtube.com/watch?v=xQ79ysnrzUk)\
*. [Hand-Eye Calibration: MoveIt Tutorials](https://ros-planning.github.io/moveit_tutorials/doc/hand_eye_calibration/hand_eye_calibration_tutorial.html)\
*. [ROS camera_calibration package](http://wiki.ros.org/camera_calibration)\
*. [OpenCV camera calibration tutorial](https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html)
