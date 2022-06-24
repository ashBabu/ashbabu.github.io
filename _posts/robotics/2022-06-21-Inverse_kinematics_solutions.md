---
layout: post
title: "Inverse Kinematics Solutions: Analytic and Optimization Based Approaches"
date: 2022-06-21 11:30:00-0400
description: How to generate solve for forward kinematics
categories: Robotics Programming
---

We have solved the symbolic [forward kinematics](/blog/2022/Generating_Symbolic_Expression_for_Forward_Kinematics/) and generated the following expressions in one of the previous articles
\begin{align}
     x &= 0.2\cos(q1) + 0.2\cos(q1 - q2) + 0.25\cos(q1 - q2 + q3) + 0.1415 \nonumber \newline
     y &= 0.2\sin(q1) + 0.2\sin(q1 - q2) + 0.25\sin(q1 - q2 + q3)  \nonumber
\end{align}

To solve these equations, I am gonna discuss two methods, viz. the analytic way which is faster but very difficult to generate equations for robots with several joints and the more general optimization based approach.

#### Analytic solution of IK
Let $$ a1 = 0.2, a2 = 0.25, a3 = 0.1415 $$

$$(x - a3)^2 + y^2 = 2a1^2 \cos q2 + 2a1a2 \cos q3 +2a1a2 \cos(q2 - q3) + 2a1^2 + a2^2$$

Let A $$ = (x - a3)^2 + y^2 - 2a1^2 \cos q2 - 2a1^2 - a2^2$$

B $$ = 2a1a2 (\cos q3 + \cos(q2 - q3)) $$
$$ = 2a1a2 (\cos q3 + \cos q2 \cos q3 + \sin q2 \sin q3) $$
$$ = 2a1a2 (\cos q3 (1+ \cos q2) + \sin q2 \sin q3) $$

Mutiplying and diving the above by $$ \sqrt{(1+ \cos q2)^2 + \sin^2 q2} = \sqrt{2(1 + \cos q2)}$$, we get

B $$ = 2a1a2 \times \sqrt{2(1 + \cos q2)} (\cos q3 \sin \theta + \cos \theta \sin q3) $$
$$ = 2a1a2 \times \sqrt{2(1 + \cos q2)} \sin (\theta + q3) $$

where $$ \sin \theta = \frac{1+ \cos q2}{\sqrt{2(1 + \cos q2)}},  \cos \theta = \frac{\sin q2}{\sqrt{2(1 + \cos q2)}} $$ and hence $$\tan \theta = \frac{1+ \cos q2}{\sin q2}$$

$$ B = A $$

$$  2a1a2 \times \sqrt{2(1 + \cos q2)} \sin (\theta + q3) = A $$

$$ q3 + \theta = \arcsin( \frac{A}{2a1a2 \times \sqrt{2(1 + \cos q2)}} )$$

Let $$ \phi = \arcsin( \frac{A}{2a1a2 \times \sqrt{2(1 + \cos q2)}} )$$

The soultion then would be $$q3+\theta = (\phi + 2n\pi, (\pi - \phi) + 2n\pi)$$ as $$\sin U = \sin (\pi-U)$$ and n = 0, 1, 2....

q3 becomes $$ \phi + 2n\pi - \theta, (\pi - \phi) + 2n\pi - \theta$$

For n = 0 and 1, the solution for q3 would be

$$ \phi - \theta, \pi - \phi - \theta, 2\pi+\phi - \theta, 3\pi - \phi - \theta $$

The above is an expression for q3 in terms of q2. Similarly (without going into too much details), we can write

$$(x - a3 - a2 \cos (q1 - q2 +q3))^2 + (y - a2 \cos (q1 - q2 + q3))^2 = 2a1^2 (1 + \cos q2) $$

$$ (x - a3) \cos q1 +  y \sin q1 = ( x^2 + y^2 - 2xa3 + a3^2 -a2^2 -2a1a2\cos q3)/2a1$$

From the above, $$ q1- q2 + q3 + \beta = \arcsin( \frac{K}{\sqrt{(x - a3)^2 + y^2}}) $$
where $$K = \frac{x^2 + y^2 - 2xa3 + a3^2 + a2^2 -2a1^2(1 + \cos q2)}{2a2}, \tan \beta = \frac{x - a3}{y}$$ and (x, y) are the coordinates of the target. 

This is coded and is available [here](https://github.com/ashBabu/moveit_kinematics_plugin/blob/master/scripts/analytic_ik.py)

#### Optimization based IK
A simple approach is adopted here to convey the idea. All optimization problems requires a cost function to minimize. We construct this as 
\begin{align}
     Min~ Z = (FK - Target)^2 = ((FK)_x - Target_x)^2 + ((FK)_y - Target_y)^2
\end{align}
where $$FK$$ and $$Target$$ respectively are the forward kinematics and the co-ordinates of the target vector in cartesian space. The $$FK$$ implicitly encodes the joint values which could be found by algorithm like `BFGS`, `fmincon`, `SLSQP` etc. This is the general idea and there are several criteria that can be added to the optimization function to achieve other desirable effects like minimum joint rotations. The the optimization function becomes
\begin{align}
     Min~ Z = (FK - Target)^2 + q1^2 + q2^2 + ....
\end{align}

The optimization procedure could be sped up by providing the [jacobian](/blog/2019/Jacobian/) (gradient function) and the implementation of one such is provided in [optim_ik](https://github.com/ashBabu/moveit_kinematics_plugin/blob/master/scripts/optim_ik.py). The subject of optimization is very vast and the brief discussion provided here is probably not enough to understand the whole process but is beyond the scope of this article. Another use of optimization is [camera calibration](/blog/2018/Camera-Calibration/). It sounds very promising to find solutions using optimization but takes a bit of time and has the biggest problem of finding [local minima](https://en.wikipedia.org/wiki/Maxima_and_minima). The quality of the initial guess to start the optimization process decides where it's going to end up and requires thorough knowledge to set things up. [KDL](http://wiki.ros.org/kdl), [Trac-ik](http://wiki.ros.org/trac_ik) are examples of kinematics library which use optimization to solve the inverse kinematics problem.
