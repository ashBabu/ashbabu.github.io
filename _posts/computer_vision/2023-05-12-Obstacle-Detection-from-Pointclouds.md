---
layout: post
title: Obstacle Detection from Pointclouds on Raspberry Pi
date: 2023-05-12 20:22:00-0500
description: realsense camera, crop box filter from PCL, ground segmentation
youtubeId: 90K8CWjifUs
categories: Computer-Vision
---

The aim here is to detect obstacles using stereo cameras which are just in front of the camera by segmenting the pointclouds into ground and the rest as obstacles. I am using intel realsense d435i for this task. The implementation can be done in `ROS` or `ROS2`. This is done on a Ubuntu linux system.

###  Pre-requisites
* Method 1:
    * `sudo apt-get install ros-$ROS_DISTRO-realsense2*` would install both `librealsense` and `realsense-ros`
* Method 2: For some reason, if the above dont work
    * Install the librealsense library on Raspberry Pi by following this [link](/blog/2021/Intel-realsense-cam-on-RaspberryPi/)
    * Install the realsense [ROS Wrapper](https://github.com/IntelRealSense/realsense-ros/releases/tag/2.3.2)
* `sudo apt install ros-$ROS_DISTRO-rtabmap-ros`

The above would install all the required packages for ROS or ROS2. 


### ROS1 specific implementation
* Launch the realsense camera as follows

```bash
<launch>
   <node pkg="tf" type="static_transform_publisher" name="base_2_camera" args="0 0 0 0 0 0 1 base_link camera_link 100" />
    <!--     <node pkg="tf" type="static_transform_publisher" name="cam_screw_2_camera" args="0 0 0 0 0 0 1 camera_bottom_screw_frame camera_link 100" /> -->
    <param name="robot_description" command="$(find xacro)/xacro '$(find realsense2_description)/urdf/test_d435_camera.urdf.xacro' use_nominal_extrinsics:=false"/>


    <arg name="fps"                 default="30" />
    <arg name="width"               default="640" />  <!-- 640 x 480  normal-->
    <arg name="height"              default="480" />
    <arg name="enable_depth"        default="true"/>
    <arg name="enable_infra1"       default="false"/>
    <arg name="enable_infra2"       default="false"/>
    <arg name="enable_color"        default="true"/>

    <include
        file="$(find realsense2_camera)/launch/rs_camera.launch">
        <arg name="depth_fps"           value="$(arg fps)"/>
        <arg name="infra_fps"           value="$(arg fps)"/>
        <arg name="color_fps"           value="$(arg fps)"/>
        <arg name="enable_gyro"         value="false"/>
        <arg name="enable_accel"        value="false"/>
        <!-- <arg name="filters"             value="pointcloud"/> -->

        <arg name="depth_width"         value="$(arg width)"/>
        <arg name="depth_height"        value="$(arg height)"/>
        <arg name="enable_depth"        value="$(arg enable_depth)"/>

        <arg name="infra_width"         value="$(arg width)"/>
        <arg name="infra_height"        value="$(arg height)"/>
        <arg name="enable_infra1"       value="$(arg enable_infra1)"/>
        <arg name="enable_infra2"       value="$(arg enable_infra2)"/>

        <arg name="color_width"         value="$(arg width)"/>
        <arg name="color_height"        value="$(arg height)"/>
        <arg name="enable_color"        value="$(arg enable_color)"/>

    </include> 

</launch>
```

* This will produce `/camera/depth/image_rect_raw`, `/camera/depth/camera_info`, `/camera/depth/color/points` topics which are fed to `point_cloud_xyz` nodelet (for downsampling and voxelizing) as

```bash
  <node pkg="nodelet" type="nodelet" name="points_xyz" args="standalone rtabmap_util/point_cloud_xyz">
       <remap from="depth/image"         to="/camera/depth/image_rect_raw"/>
       <remap from="depth/camera_info"   to="/camera/depth/camera_info"/>
       <remap from="cloud"               to="/camera/depth/color/points"/>
       <param name="decimation"  type="double" value="4"/>
       <param name="voxel_size"  type="double" value="0.05"/>
       <param name="approx_sync" type="bool" value="false"/>
    </node>
```
* The above will publish downsampled and voxel filtered pointcloud under the topic `camera/depth/color/points`
* The cropping of the pointcloud is achieved by using the point cloud library's (PCL) [cropbox filter](https://pointclouds.org/documentation/classpcl_1_1_crop_box_3_01pcl_1_1_p_c_l_point_cloud2_01_4.html). This would be as follows
  * subscribe to the pointcloud2 topic`camear/depth/color/points`
  ```bash
  void FilterNearbyObstacles::pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // Container for original & filtered data
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr croppedCloud (new pcl::PointCloud<pcl::PointXYZ>);
    cropboxFilter(*cloud, *croppedCloud);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 cropped_cloud;
    pcl::toROSMsg(*croppedCloud, cropped_cloud);
    // Publish the data
    croppedCloudPub_.publish(cropped_cloud);
}

    void FilterNearbyObstacles::cropboxFilter(const pcl::PCLPointCloud2 & inCloud, pcl::PointCloud<pcl::PointXYZ> & filteredCloud)
{
    // https://stackoverflow.com/questions/45790828/remove-points-outside-defined-3d-box-inside-pcl-visualizer
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr input_clou = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromPCLPointCloud2(inCloud,*input_cloud);

    pcl::CropBox<pcl::PointXYZ> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(x_min_, -10.0, z_min_, 1.0));
    boxFilter.setMax(Eigen::Vector4f(x_max_, 10.0, z_max_, 1.0));
    boxFilter.setInputCloud(input_cloud);
    boxFilter.filter(filteredCloud);
}
```
where `x` and `z` are the dimensions of the box filter. This also publishes the box filtered pointcloud under the topic `/cropped_pointcloud`.

* The `/cropped_pointcloud` is then fed to the `obstacles_detection` nodelet as 

```bash
 <node pkg="nodelet" type="nodelet" name="rtabmap_obstacles_detection" args="standalone rtabmap_util/obstacles_detection stereo_nodelet">
        <remap from="cloud" to="/cropped_pointcloud"/>
        <param name="frame_id"             type="string" value="camera_link"/>
        <param name="wait_for_transform"   type="bool"   value="true"/>
        <param name="Grid/MaxGroundHeight"   type="double"   value="0.01"/>
    </node>
```
The above will produce obstacles under the topic `/obstacles` by segmenting out the ground

### ROS2 Implementation
* Install `realsense` and `rtabmap-ros` as mentioned above. 
* The `realsense` camera is launched as 

```bash
import os
import launch
import launch_ros.actions
from launch.substitutions import Command
from launch.actions import IncludeLaunchDescription
from ament_index_python import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_path
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    robot_description = ParameterValue(
        Command(['xacro ', str(get_package_share_path('realsense2_description') / 'urdf/test_d435_camera.urdf.xacro')]),
        value_type=str)

    rs_cam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch/rs_launch.py')),
        launch_arguments={
            "depth_module.profile": "640x480x30",
           # "pointcloud.enable": "True",
        }.items()
    )

    return launch.LaunchDescription([
        rs_cam,
        launch_ros.actions.Node(package='robot_state_publisher', executable='robot_state_publisher',
                                parameters=[{'robot_description': robot_description}])
    ])
```

* The obstacle detection part is launched as a [ros2 component](https://docs.ros.org/en/humble/Tutorials/Intermediate/Composition.html)

```bash
"""Launch a obstacle segmentation in a component container."""

import launch
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer


def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
        name='obstacle_detection',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='rtabmap_util',
                plugin='rtabmap_util::PointCloudXYZ',
                name='points_xyz_rt',
                remappings=[
                            ("depth/image", "/camera/depth/image_rect_raw"),
                            ("depth/camera_info", "/camera/depth/camera_info"),
                            ("cloud", "/camera/depth/color/points")
                            ],
                parameters=[
                            {"decimation": 4},
                            {"voxel_size": 0.05},
                            {"approx_sync": False}
                            ]
                ),
            ComposableNode(
                package='rtabmap_util',
                plugin='rtabmap_util::ObstaclesDetection',
                name='obstacle_detection_rt',
                remappings=[
                            ("cloud", "/cropped_pointcloud")
                            ],
                parameters=[
                            {"frame_id": "camera_link"},
                            {"wait_for_transform": 1.0},
                            {"Grid/MaxGroundHeight": "0.04"}
                            ]
                )
        ],
        output='screen',
    )
    return launch.LaunchDescription([container])
```
* Use the cropbox filter as above.

**Notes**
* Launching realsense camera with the launch_arguments `"pointcloud.enable": "True"` on raspberry pi will not publish pointclouds. This argument infact slows down the publishing rate of depth images from the specified rate of 30 to around 8 Hz. However if the `pointcloud.enable` argument is removed, then the rate of publishing of depth images is at the specified 30 Hz itself and by using either the `plugin='rtabmap_util::PointCloudXYZ'` or [depth_image_proc](http://wiki.ros.org/depth_image_proc), the pointclouds could be published at around the same 30 Hz rate.
<br />
{% include youtubePlayer.html id=page.youtubeId %}



