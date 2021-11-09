# **Ubuntu install of ROS Noetic**

> NOTE: This instruction focuses on the perfomance installation and running commands. For more detail, you can read pdf book [ROS Robot Programming (35.6 MB)](https://www.robotis.com/service/download.php?no=719) and go to [ROS Official Tutorials](https://wiki.ros.org/ROS/Tutorials).

<br>

### Previous: [13. ROS Tools #1: 3D Visualization Tool (RViz)](13-ROS-Tools-(1)-3D-Visualization-Tool-(RViz).md)

<br>

# 15. Basic ROS Programming #1: Creating and Running Publisher and Subscriber Nodes

The publishers and subscribers used in ROS message communication can be compared with a transmitter and receiver. In ROS, the transmitter is called publisher, and the receiver is called subscriber. This section aims to create a simple message file, and create and run Publisher and Subscriber nodes.

## 1. Creating a Package

The following command creates a ‘ros_tutorials_topic’ package. This package is dependent on the ‘message_generation’, ‘std_msgs’, and ‘roscpp’ packages, as they are appended as dependency options followed by the custom package name. The ‘message_generation’ package will be required to create a new message. ‘std_msgs’ is the ROS standard message package and ‘roscpp’ is the client library to use C/C++ in ROS. These dependent packages can be included while creating the package, but they can also be added after creating the ‘package.xml’ in the package
folder.












