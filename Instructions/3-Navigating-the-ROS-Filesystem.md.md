# **Ubuntu install of ROS Noetic**

> NOTE: This instruction focuses on the perfomance installation and running commands. For more detail, you can read pdf book [ROS Robot Programming (35.6 MB)](https://www.robotis.com/service/download.php?no=719) and go to [ROS Official Tutorials](https://wiki.ros.org/ROS/Tutorials).

<br>

### Previous: [2. Create a ROS Workspace](2-Create-a-ROS-Workspace.md.md)

<br>

# 3. Navigating the ROS Filesystem

## 3.1 Prerequisites

The new build system for ROS is "catkin", while "rosbuild" is the old ROS build system which was replaced by catkin. If you are new to ROS, choose catkin.

For this tutorial we will inspect a package in ros-tutorials, please install it using

    sudo apt-get install ros-<distro>-ros-tutorials

Replace '<distro>' (including the '<>') with the name of your [ROS distribution](http://wiki.ros.org/Distributions) (e.g. indigo, kinetic, lunar etc.)

## 3.2 Quick Overview of Filesystem Concepts

- **Packages:** Packages are the software organization unit of ROS code. Each package can contain libraries, executables, scripts, or other artifacts.

- **Manifests ([package.xml](http://wiki.ros.org/catkin/package.xml)):** A manifest is a description of a package. It serves to define dependencies between packages and to capture meta information about the package like version, maintainer, license, etc...

## 3.3 Filesystem Tools

Code is spread across many ROS packages. Navigating with command-line tools such as ls and cd can be very tedious which is why ROS provides tools to help you.

### 3.3.1 Using rospack

[rospack](http://wiki.ros.org/rospack) allows you to get information about packages. In this tutorial, we are only going to cover the find option, which returns the path to package.

Usage:

    rospack find [package_name]

Example:

    rospack find roscpp

would return:

    YOUR_INSTALL_PATH/share/roscpp

If you installed ROS Noetic from apt on Ubuntu Linux you would see exactly:

    /opt/ros/noetic/share/roscpp

### 3.3.2 Using roscd

[roscd](http://wiki.ros.org/rosbash#roscd) is part of the [rosbash](http://wiki.ros.org/rosbash) suite. It allows you to change directory ([cd](https://ss64.com/bash/cd.html)) directly to a package or a stack.

Usage:

    roscd <package-or-stack>[/subdir]

To verify that we have changed to the roscpp package directory, run this example:

    roscd roscpp

Now let's print the working directory using the Unix command [pwd](https://ss64.com/bash/pwd.html):

    pwd

You should see:

    YOUR_INSTALL_PATH/share/roscpp

You can see that `YOUR_INSTALL_PATH/share/roscpp` is the same path that `rospack` find gave in the previous example.

In this case, you should see output is:

    /opt/ros/noetic/share/roscpp

> The term "noetic' is one of [ROS distribution](http://wiki.ros.org/Distributions).

Note that [roscd](http://wiki.ros.org/rosbash#roscd), like other ROS tools, will only find ROS packages that are within the directories listed in your [ROS_PACKAGE_PATH](http://wiki.ros.org/ROS/EnvironmentVariables#ROS_PACKAGE_PATH). To see what is in your [ROS_PACKAGE_PATH](http://wiki.ros.org/ROS/EnvironmentVariables#ROS_PACKAGE_PATH), type:

    echo $ROS_PACKAGE_PATH

Your [ROS_PACKAGE_PATH](http://wiki.ros.org/ROS/EnvironmentVariables#ROS_PACKAGE_PATH) should contain a list of directories where you have ROS packages separated by colons. A typical [ROS_PACKAGE_PATH](http://wiki.ros.org/ROS/EnvironmentVariables#ROS_PACKAGE_PATH) might look like this:

    /opt/ros/noetic/share

Similarly to other environment paths, you can add additional directories to your [ROS_PACKAGE_PATH](http://wiki.ros.org/ROS/EnvironmentVariables#ROS_PACKAGE_PATH), with each path separated by a colon ':'.

#### 3.3.2.1 Subdirectories

[roscd](http://wiki.ros.org/rosbash#roscd) can also move to a subdirectory of a package or stack.

Try:

    roscd roscpp/cmake
    pwd

You should see:

    YOUR_INSTALL_PATH/share/roscpp/cmake

A example of output could be:

    /opt/ros/noetic/share/roscpp/cmake

### 3.3.3 roscd log

`roscd log` will take you to the folder where ROS stores log files. Note that if you have not run any ROS programs yet, this will yield an error saying that it does not yet exist.

Open a Terminal and try:

    roscore

Then open a new Terminal and type:

    roscd log

Output:

    .ros/log/<id of ROS program>

Example:

    .ros/log/ab7c19da-1f7c-11ec-a02d-5b0baf5c9082

> Note: You also see the ID in the Terminal runing ROS program.

### 3.3.4 Using rosls

[rosls](http://wiki.ros.org/rosbash#rosls) is part of the [rosbash](http://wiki.ros.org/rosbash) suite. It allows you to [ls](https://ss64.com/bash/ls.html) directly in a package by name rather than by absolute path.

Usage:

    rosls <package-or-stack>[/subdir]

Example:

    rosls roscpp_tutorials

would return:

    cmake launch package.xml  srv

### 3.3.5 Tab Completion

It can get tedious to type out an entire package name. In the previous example, `roscpp_tutorials` is a fairly long name. Luckily, some ROS tools support [TAB completion](https://en.wikipedia.org/wiki/Command-line_completion).

Start by typing:

    roscd roscpp_tut<<< now push the TAB key >>>

After pushing the **TAB** key, the command line should fill out the rest:

    roscd roscpp_tutorials/

This works because `roscpp_tutorials` is currently the only ROS package that starts with `roscpp_tut`.

Now try typing:

    roscd tur<<< now push the TAB key >>>

After pushing the **TAB** key, the command line should fill out as much as possible:

    roscd turtle

However, in this case there are multiple packages that begin with turtle. Try typing **TAB** another time. This should display all the ROS packages that begin with turtle:

    turtle_actionlib/  turtlesim/         turtle_tf/

On the command line you should still have:

    roscd turtle

Now type an s after turtle and then push **TAB**:

    roscd turtles<<< now push the TAB key >>>

Since there is only one package that starts with turtles, you should see:

    roscd turtlesim/

If you want to see a list of all currently installed packages, you can use tab completion for that as well:

    rosls <<< now push the TAB key twice >>>

## 3.4 Review

You may have noticed a pattern with the naming of the ROS tools:

- rospack = ros + pack(age)
- roscd = ros + cd
- rosls = ros + ls

This naming pattern holds for many of the ROS tools.

<br>

### Next: [4. Create a package](4-Creating-a-ROS-Package.md.md)







