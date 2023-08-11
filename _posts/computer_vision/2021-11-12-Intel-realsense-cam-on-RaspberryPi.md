---
layout: post
title: Intel realsense camera on Raspberry Pi
date: 2021-11-12 20:22:00-0500
description: realsense camera, crop box filter from PCL, ground segmentation
categories: Computer-Vision
---

### Prerequisites
* Microsd card,
* Microsd-SD adapter 
* A computer that can read and flash the microsd card.

### Procedure
Read this [Using depth camera with Raspberry Pi 3](https://dev.intelrealsense.com/docs/using-depth-camera-with-raspberry-pi-3) to get a general idea.

* In the computer, `sudo apt-get install rpi-imager or snap install rpi-imager`
* Open the imager and use the dropdown menu to select the OS version (Ubuntu server 20.04 for me) and where to write to (microSD card) and write
* Put the microusb in the rPi slot and it would take on to the installation screen and will take a few minutes to finish.
* For installing `librealsense`, follow the steps **without** the kernal patch as shown in [Linux installation](https://github.com/IntelRealSense/librealsense/blob/development/doc/installation.md) or download the last version of `librealsense` (for ROS1) supported is [v2.50.0](https://github.com/IntelRealSense/librealsense/releases/tag/v2.50.0).
* Build it with `cmake ../ -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true -DFORCE_RSUSB_BACKEND=ON`
* `make -j$(nproc)`
* `sudo make install`

### Observations
The rPi was able to run the `realsense-viewer` and the camera showed both RGB and depth images. After running it for a few minutes, the rPi automatically rebooted. I suspect this could be because of overheating as the rPi was very hot to touch. So I added a fan and also a [battery](https://www.amazon.co.uk/dp/B09QRS666Y?ref_=cm_sw_r_apan_dp_M0NDDE6Q26BRT4SX2F6Z) for portability.

<img style="float: left;" title="Camera calibration" src="/assets/img/computer-vision/rpi.jpg" alt="Camera Calibration" width="750" height="400"/>



