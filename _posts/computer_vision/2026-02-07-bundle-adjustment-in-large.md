---
layout: post
title: "Revisiting Bundle Adjustment with Ceres"
date: 2026-02-07 22:25:00-0500
description: 
# youtubeId: 90K8CWjifUs
categories: Computer-Vision
---

### Camera Projection Model
The projection from 3D to 2D can be represented as
\begin{align}
    s
    \begin{bmatrix}
        u  \nonumber \newline
        v  \nonumber \newline
        1 
    \end{bmatrix} = [K ~R~ t]_{3\times4}
     \begin{bmatrix}
        X  \nonumber \newline
        Y  \nonumber \newline
        Z  \tag{1}
        1 
    \end{bmatrix}
\end{align}
where $s$ is the scale factor, $(u, v)$ are the projected pixel coordinates, $K, R, t$ are the camera intrinsics, Rotation (3 x 3) and translation (3 x 1)(extrinsics) and $(X, Y, Z)$ are the 3D co-ordinates of the world point or landmark. 

The matrix K for a pinhole camera is 

\begin{align}
    \begin{bmatrix}
        f_x & 0 & c_x \nonumber \newline
        0 & f_y & c_y \nonumber \newline
        0 & 0 & 1 
    \end{bmatrix} 
    \label{eq:intrinsic}
\end{align}

The $[K~R~t]$ together is called the Projection matrix, $P_{3\times4}$. 
Other projection models include having distortion coefficients,

\begin{align}
    P  &=  R * X + t ~~~~ \textnormal{(conversion from world to camera coordinates)} \nonumber \newline
    p  &= -\frac{P}{P.z}   ~~~~  \textnormal{(perspective division, -ve bcoz image plane is behind camera)} \nonumber \newline
    p' &=  f * r(p) * p   ~~~~ \textnormal{(conversion to pixel coordinates)} \nonumber \newline
    r(p) &= 1.0 + k1 * ||p||^2 + k2 * ||p||^4. \nonumber
\end{align}
where $P.z$ is the third (z) coordinate of $P$. In the last equation, $r(p)$ is a function that computes a scaling factor to undo the radial distortion:


{% include figure.liquid path="assets/img/computer-vision/cam_wd_pts.jpeg" class="img-fluid rounded z-depth-1" zoomable=true caption="Fig.1. Bundle Adjustment Schematic." 
   id="ba-schematic" %}

Given the true observation (camera pixels) values (LHS of [Eq. 1](#1). Using RHS of the [Eq. 1](#1), we get the predicted value for the pixel coordinates. The error is therefore calculated as 

\begin{align}
    e = \begin{bmatrix}
            u_{predicted} - u_{observed} \nonumber \newline
            v_{predicted} - v_{observed}
        \end{bmatrix}
\end{align}

In [Fig 1](#ba-schematic), camera 1 sees the points P1, P2, P3 and P4, camera 2 sees the points P3, P4, P5 and so on and so forth. So the error will have the components $e_{11}, e_{12}, e_{13}, e_{14}, e_{23}, e_{24}, e_{25}$ etc. Here, $e_{ij}$ has $i$ as the camera index and $j$ as the landmark index. The cost function for an optimization problem can be formulated as

\begin{align}
    \frac{1}{2} \Sigma || e_{ij} || ^2
\end{align}

To solve such a non-linear optimization problem, Ceres {% cite ceres-solver %} is used. The dataset has been obtained from <a href="https://grail.cs.washington.edu/projects/bal/"> Grail lab, University of Washington </a>

### Triangulation
Let the rows of $P$ be $p_1$, $p_2$ and $p_3$ and $\boldsymbol{X}=(X, Y, Z, 1)^T$. Therefore, 
\begin{align}
    su &= p_1X \nonumber \newline
    sv &= p_2X \nonumber \newline
    s &= p_3X
\end{align}
Eliminating $s$, we get 
\begin{align}
    (up_3 - p_1)\boldsymbol{X} &= 0 \nonumber \newline
    (vp_3 - p_2)\boldsymbol{X} &= 0
\end{align}
This means that for each observed point we get a pair of equations. If there are multiple observations for the same point from different camera poses, we can stack these together to form an eigenvalue problem of the form
\begin{align}
    A \boldsymbol{X } = 0
\end{align}

The matrix $A$ can be solved using singular value decomposition (SVD)

A very good book to refresh the concepts can be found {% cite slambook2017 %}
### Code
A `utilities.hpp` is created as follows
```bash
#pragma once

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <Eigen/Dense>
#include <fstream>

namespace bundle_adjustment {
struct CameraParams {
  /*
  params layout:
  [0] rx (angle-axis)
  [1] ry (angle-axis)
  [2] rz (angle-axis)
  [3] tx (translation)
  [4] ty (translation)
  [5] tz (translation)
  [6] focal_length   
  [7] k1 (radial distortion)
  [8] k2 (radial distortion)

  */
  double params[9];
};

struct WorldPoint {
  double xyz[3];
};

struct Observation {
  int camera_index;
  int point_index;
  double u;
  double v;
};

/*
Triangulation using Direct Linear Transformation
*/
WorldPoint TriangulateDLT(const std::vector<Observations>& obs) {
  Eigen::MatrixXd A(2 * obs.size(), 4);

  for (size_t i = 0; i < obs.size(); ++i) {
    const auto P = obs[i].P;
    double u = obs[i].u;
    double v = obs[i].v;

    // Row 1: u * P3 - P1
    A.row(2 * i) = u * P.row(2) - P.row(0);
    // Row 2: v * P3 - P2
    A.row(2 * i + 1) = v * P.row(2) - P.row(1);
  }

  // Solve A*X = 0 using SVD
  // The solution is the eigenvector corresponding to the smallest eigenvalue
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
  Eigen::Vector4d X_homo = svd.matrixV().col(3);

  // Convert from homogeneous to Euclidean
  WorldPoint world_pts;
  Eigen::Vector3d wp(X_homo.head<3>() / X_homo(3));
  world_pts.xyz[0] = wp(0);
  world_pts.xyz[1] = wp(1);
  world_pts.xyz[2] = wp(2);

  return world_pts;
}

struct BALProblem {
  int num_cameras;
  int num_points;
  int num_observations;

  std::vector<Observation> observations;
  std::vector<CameraParams> cameras;
  std::vector<WorldPoint> points;

  bool LoadFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) return false;

    // 1. Read the header
    file >> num_cameras >> num_points >> num_observations;

    // 2. Read the observations
    observations.resize(num_observations);
    for (int i = 0; i < num_observations; ++i) {
      file >> observations[i].camera_index >> observations[i].point_index >>
          observations[i].u >> observations[i].v;
    }

    // 3. Read the camera parameters (9 per camera)
    cameras.resize(num_cameras);
    for (int i = 0; i < num_cameras; ++i) {
      for (int j = 0; j < 9; ++j) {
        file >> cameras[i].params[j];
      }
    }

    // 4. Read the point parameters (3 per point)
    points.resize(num_points);
    for (int i = 0; i < num_points; ++i) {
      for (int j = 0; j < 3; ++j) {
        file >> points[i].xyz[j];
      }
    }

    return true;
  }

  void WriteToPLY(const std::string& filename) {
    std::ofstream ofs(filename);
    ofs << "ply\nformat ascii 1.0\n"
        << "element vertex " << num_points << "\n"
        << "property float x\nproperty float y\nproperty float z\n"
        << "end_header\n";

    for (const auto& p : points) {
        ofs << p.xyz[0] << " " << p.xyz[1] << " " << p.xyz[2] << "\n";
    }
}
};

struct ReprojectionError {
  ReprojectionError(double observed_x, double observed_y)
      : observed_x_(observed_x), observed_y_(observed_y) {}

  template <typename T>
  bool operator()(const T* const camera, const T* const point,
                  T* residuals) const {
    const T* angle_axis = camera + 0;
    const T* translation = camera + 3;
    const T& focal = camera[6];
    const T& k1 = camera[7];
    const T& k2 = camera[8];

    T p[3];
    ceres::AngleAxisRotatePoint(angle_axis, point, p);

    // Translate the point
    p[0] += translation[0];
    p[1] += translation[1];
    p[2] += translation[2];

    // Project to image plane
    T xp = -p[0] / p[2];  // negative because cam img plane is behind
    T yp = -p[1] / p[2];

    T r2 = xp * xp + yp * yp;
    T rp = T(1.0) + k1* r2 + k2* r2* r2;

    // Apply intrinsics
    T predicted_x = focal * rp * xp;
    T predicted_y = focal * rp * yp;

    // 5. Compute residuals
    residuals[0] = predicted_x - T(observed_x_);
    residuals[1] = predicted_y - T(observed_y_);

    return true;
  }

  // Factory to hide the construction of the CostFunction object
  static ceres::CostFunction* Create(double observed_x, double observed_y) {
    return new ceres::AutoDiffCostFunction<ReprojectionError, 2, 9,
                                           3>(  // 2 residuals, 9 camera params,
                                                // 3 point params
        new ReprojectionError(observed_x, observed_y));
  }

  double observed_x_;
  double observed_y_;
};

}  // namespace bundle_adjustment
```

The `main.cpp` is as follows

```bash
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "utilities.hpp"

namespace ba = bundle_adjustment;

int main(int argc, char** argv) {
  ba::BALProblem data;
  // if (data.LoadFile("problem-16-22106-pre.txt")) {
  if (data.LoadFile("problem-135-90642-pre_dubrovnik.txt")) {
    std::cout << "Loaded " << data.num_cameras << " cameras and "
              << data.num_points << " points." << std::endl;
  }

  ceres::Problem problem;
  for (const auto& obs : data.observations) {
    double* cam_params = data.cameras[obs.camera_index].params;
    double* landmarks = data.points[obs.point_index].xyz;
    double u = obs.u;
    double v = obs.v;

    // Create Cost Function
    ceres::CostFunction* cost_function = ba::ReprojectionError::Create(u, v);

    // Add Residual Block
    problem.AddResidualBlock(
        cost_function,
        new ceres::HuberLoss(1.0),  // Robust loss against outliers
        cam_params, landmarks);
  }
  problem.SetParameterBlockConstant(data.cameras[0].params);

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  options.num_threads = 16; // Use multiple cores
  options.preconditioner_type = ceres::SCHUR_JACOBI; 
  options.max_num_iterations = 200;

  ceres::Solver::Summary summary;
  std::cout << "Solving..." << std::endl;
  
  data.WriteToPLY("initial_dubrovnik.ply");
  
  ceres::Solve(options, &problem, &summary);

  data.WriteToPLY("optimized_dubrovnik.ply");

  std::cout << summary.FullReport() << "\n";
  return 0;
}
```

The `CMakeLists.txt` 
```bash
cmake_minimum_required(VERSION 3.5)
project(ba_uni_washington)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Ceres REQUIRED)
find_package(Eigen3)

include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
)

add_executable(bundle_adjustment 
    bundle_adjustment.cpp 
)
target_link_libraries(bundle_adjustment 
    PRIVATE ceres
    ${EIGEN3_LIBRARIES}
)
install(TARGETS bundle_adjustment 
    DESTINATION lib/${PROJECT_NAME}
)
```
### Results

<img style="float: left;" title="Camera calibration" src="/assets/img/computer-vision/bundle_adjustment/bundle_adjustment.png" alt="bundle_adjustment" width="900" height="400"/>

<div style="clear: both; margin-bottom: 50px;"></div>
### References 
<div class="publications">
  {% bibliography --cited %}
</div>