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

Open tetminal and run commands:

    cd ~/catkin_ws/src   
    catkin_create_pkg ros_tutorials_topic message_generation std_msgs roscpp

When the package is created, the ‘ros_tutorials_topic’ package folder is created in the ‘~/catkin_ws/src’ folder. In this package folder, the ‘CMakeLists.txt’ and ‘package.xml’ files are created along with default folders. You can inspect it with the ‘ls’ command as below, or check the inside of the package using the GUI-based Nautilus, which is similar to Windows File Explorer.

Run commands:

    cd ros_tutorials_topic
    ls

# 2. Modifying the Package Configuration File (package.xml)

The ‘package.xml’ file, one of the required ROS configuration files, is an XML file containing the package information such as the package name, author, license, and dependent packages. Let’s open the file using an editor (such as gedit, vim, emacs, etc.) with the following command and modify it for the current node.

    gedit package.xml

The following code shows how to modify the ‘package.xml’ file to match the package you created. Personal information will be included in the content, so you can modify it as you wish.

    <package format="2">
        <name>ros_tutorials_topic</name>
        <version>0.1.0</version>
        <description>ROS tutorial package to learn the topic</description>
        <!--  One maintainer tag required, multiple allowed, one person per tag  -->
        <!--  Example:   -->
        <!--  <maintainer email="jane.doe@example.com">Jane Doe</maintainer>  -->
        <maintainer email="trung.lyduc18@gmail.com">Duc Trung</maintainer>
        <!--  One license tag required, multiple allowed, one license per tag  -->
        <!--  Commonly used license strings:  -->
        <!--    BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3  -->
        <license>Apache License 2.0</license>
        <!--  Url tags are optional, but multiple are allowed, one per tag  -->
        <!--  Optional attribute type can be: website, bugtracker, or repository  -->
        <!--  Example:  -->
        <!--  <url type="website">http://wiki.ros.org/ros_tutorials_topic</url>  -->
        <!--  Author tags are optional, multiple are allowed, one per tag  -->
        <!--  Authors do not have to be maintainers, but could be  -->
        <!--  Example:  -->
        <author email="trung.lyduc18@gmail.com">Duc Trung</author>
        <!--  The *depend tags are used to specify dependencies  -->
        <!--  Dependencies can be catkin packages or system dependencies  -->
        <!--  Examples:  -->
        <!--  Use depend as a shortcut for packages that are both build and exec dependencies  -->
        <!--    <depend>roscpp</depend>  -->
        <!--    Note that this is equivalent to the following:  -->
        <!--    <build_depend>roscpp</build_depend>  -->
        <!--    <exec_depend>roscpp</exec_depend>  -->
        <!--  Use build_depend for packages you need at compile time:  -->
        <!--    <build_depend>message_generation</build_depend>  -->
        <!--  Use build_export_depend for packages you need in order to build against this package:  -->
        <!--    <build_export_depend>message_generation</build_export_depend>  -->
        <!--  Use buildtool_depend for build tool packages:  -->
        <!--    <buildtool_depend>catkin</buildtool_depend>  -->
        <!--  Use exec_depend for packages you need at runtime:  -->
        <!--    <exec_depend>message_runtime</exec_depend>  -->
        <!--  Use test_depend for packages you need only for testing:  -->
        <!--    <test_depend>gtest</test_depend>  -->
        <!--  Use doc_depend for packages you need only for building documentation:  -->
        <!--    <doc_depend>doxygen</doc_depend>  -->
        <buildtool_depend>catkin</buildtool_depend>
        <build_depend>roscpp</build_depend>
        <build_depend>std_msgs</build_depend>
        <build_depend>message_generation</build_depend>
        <build_export_depend>roscpp</build_export_depend>
        <build_export_depend>std_msgs</build_export_depend>
        <exec_depend>roscpp</exec_depend>
        <exec_depend>std_msgs</exec_depend>
        <exec_depend>message_runtime</exec_depend>
        <!--  The export tag contains other, unspecified, tags  -->
        <export>
        <!--  Other tools can request additional information be placed here  -->
        </export>
    </package>

































