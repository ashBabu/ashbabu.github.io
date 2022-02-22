---
layout: post
title: Octomap building in ROS
date: 2018-09-21 22:22:00-0500
description: Voxel based Octomap for scene segmentation
categories: Computer-Vision
---

###  Pre-requisites
```sudo apt-get install ros-melodic-octomap ros-melodic-octomap-msgs ros-melodic-octomap-ros ros-melodic-octomap-rviz-plugins ros-melodic-octomap-server ros-melodic-octomap-mapping ros-melodic-freenect-launch libfreenect-dev ```

### Instructions for cloning and building (Optional)
Octomap_server is used to generate and save octomaps as .bt (binary) or .ot (oct-tree) files.
* `cd catkin_ws/src`
* `git clone https://github.com/OctoMap/octomap_mapping`
* `git clone https://github.com/code-iai/iai_kinect2`
* `catkin_make`

create a catkin package 
 * `cd src/`
 * `catkin_create_pkg cameras roscpp rospy std_msgs`
 * `cd ..`
 * `catkin_make`

and add the following launch file, `start_cameras.launch` inside a launch folder

```bash
<launch>
  
     <include file="$(find kinect2_bridge)/launch/kinect2_bridge.launch">
       <arg name="base_name" value="ceiling_camera"/>
       <arg name="sensor" value="002608570547" />  <!-- Replace this line with your camera serial number -->
       <arg name="fps_limit" value="15" />
       <arg name="publish_tf" value="true" />
       <arg name="base_name_tf" value="ceiling_camera" />
     <!--  <arg name="output_frame" value="ceil" />   -->
     </include>

<node pkg="tf" type="static_transform_publisher" name="ceiling_cam_transform" args="
-0.12019394, -0.04154276,  1.53544738  0.64843474, -0.64567785,  0.28140082, -0.28887035 /world /ceiling_camera_link 100"/>  

</launch>
```

The procedures for building an octmap for visualization in `Rviz` are follows:

1. ``` roslaunch camera_setup start_cameras.launch ```  # to launch the kinect 3D sensor 
2. ```roslaunch octomap_server octomap_mapping.launch ``` # 2 things need to be changed 1) frame_id   2) remap cloud_in. check the forked octomap_mapping.launch [here](https://github.com/ashBabu/panda_moveit_config_ash/blob/master/launch/demo_promp.launch#L32)
3. ``` rosrun rviz rviz```
4. Add `MarkerArray` in `Rviz` and select the topic as `/occupied_cells_vis_array`. Set the same frame_id above (in 2) in the Global option --> Fixed frame in `Rviz`
5. ```rosrun octomap_server octomap_saver -f map_name.bt (or .ot)``` # to save the octomap

### Visualize octomap in `Rviz`
1. Close all the above
2. ``` roscore```  
3. ``` rosrun rviz rviz ```
4. ``` rosrun octomap_server octomap_server_node map_name.bt ```
5. Add `MarkerArray` in `Rviz` and select the topic as `/occupied_cells_vis_array`. Set the same frame_id as it was recorded in the Global option --> Fixed frame in Rviz. There might be a chance that the octomap wont show up. Then write frame id as `map` which should solve the problem
