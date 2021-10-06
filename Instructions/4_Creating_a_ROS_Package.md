# **Ubuntu install of ROS Noetic**

> NOTE: This instruction focuses on the perfomance installation and running commands. For more detail, you can read pdf book [ROS Robot Programming (35.6 MB)](https://www.robotis.com/service/download.php?no=719) and go to [ROS Official Tutorials](https://wiki.ros.org/ROS/Tutorials).

<br>

### Previous: [3. Navigating the ROS Filesystem](3_Navigating_the_ROS_Filesystem.md)

<br>

# 4. Creating a ROS Package

## 4.1 What makes up a catkin Package?

For a package to be considered a catkin package it must meet a few requirements:

- The package must contain a [catkin compliant package.xml](http://wiki.ros.org/catkin/package.xml) file.

    - That package.xml file provides meta information about the package.

- The package must contain a CMakeLists.txt which uses catkin.

