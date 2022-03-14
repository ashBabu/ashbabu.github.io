---
layout: post
title: Remote monitoring on a web browser
date: 2022-03-14 06:15:00-0400
description: A short intro on how to monitor robots on a browser
youtubeId: dXEoPc54iKo
categories: Robotics ROS
enable_math: true
---

<div style="text-align: justify">
This article gives a brief explanation of how remote monitoring of robots integrated into <a href="https://www.ros.org/"> ROS </a>can be done in a web browser. Here I have integrated a live camera feed with robot states. Follow this <a href="https://wiki.ros.org/ros3djs/Tutorials/VisualizingAURDF"> tutorial </a> to get a feel of how this is done. In order for streaming live camera information, the images need to be compressed and republished. To solve some of the issues that may come up, this <a href="https://github.com/RobotWebTools/ros3djs/issues/209"> link </a> could be useful. A similar but another version of remote monitoring using a HoloLens is done in one of my <a href="/projects/ros-unity/"> projects </a>.
</div>
<br />
{% include youtubePlayer.html id=page.youtubeId %}

<br />

### Resources
* [Robot Web Tools](http://robotwebtools.org/)
* [My own collections](https://github.com/ashBabu/Utilities/wiki/Remote-Monitoring:-ROS-in-WebPage)
