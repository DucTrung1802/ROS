# **Ubuntu install of ROS Noetic**

> NOTE: This instruction focuses on the perfomance installation and running commands. For more detail, you can read pdf book [ROS Robot Programming (35.6 MB)](https://www.robotis.com/service/download.php?no=719) and go to [ROS Official Tutorials](https://wiki.ros.org/ROS/Tutorials).

<br>

### Previous: [13. ROS Tools #1: 3D Visualization Tool (RViz)](13-ROS-Tools-(1)-3D-Visualization-Tool-(RViz).md)

<br>

# 14. ROS Tools #2: ROS GUI Development Tool (rqt)

Besides the 3D visualization tool RViz, ROS provides various GUI tools for robot development. For example, there is a graphical tool that shows the hierarchy of each node as a diagram thereby showing the status of the current node and topic, and a plot tool that schematizes a message as a 2D graph. Starting from the ROS Fuerte version, more than 30 GUI development tools have been integrated as the tool called rqt 6 which can be used as a comprehensive GUI tool. Furthermore, RViz has also been integrated as a plugin of rqt, making rqt an essential GUI tool for ROS.

## 1. Installing and Running rqt

If you have installed ROS with ‘ros-[ROS_DISTRO]-desktop-full’ command rqt will be installed by default. If you did not install the ‘desktop-full’ version ROS or for some reason, ‘rqt’ is not installed, then the following command will install ‘rqt’.

    sudo apt-get install ros-noetic-rqt*

The command to run rqt is as follows. You can simply type in ‘rqt’ on the terminal. For your reference, we can also run it with the node execution command ‘rosrun rqt_gui rqt_gui’.

    rqt

If we run ‘rqt’ then the GUI screen of rqt will appear as shown below. If it is the first time being launched, it will display only the menu without any content below. This is because the plugin, which is the program that is directly run by ‘rqt’, has not been specified.

![Initial_screen_of_rqt.png](../Images/Initial_screen_of_rqt.png)

The rqt menus are as follows.

- **Files:** The File menu only contains the sub-menu to close ‘rqt’.

- **Plugins:** There are over 30 plugins. Select the plugin to use.

- **Running:** The currently running plugins are shown and they can be stopped when
they are not needed.

- **Perspectives:** This menu saves operating plugins as a set and uses them later to run the same plugins.

## 2. rqt Plugins

From the ‘rqt’ menu on the top, if we select [Plugins] we can see about 30 plugins. These plugins have the following roles. Most of them are default plugins of ‘rqt’ that have very useful features. Unofficial plugins can also be added, and if necessary, we can add custom ‘rqt’ plugins that we developed for ourselves as well.

### Action

- Action Type Browser: This is a plugin to check the data structure of an action type.

### Configuration

- Dynamic Reconfigure: This is a plugin to modify the parameter value of a node.

- Launch This is a GUI plugin of roslaunch, which is useful when we cannot remember the
name or composition of roslaunch.






