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

- The package must contain a [CMakeLists.txt which uses catkin](http://wiki.ros.org/catkin/CMakeLists.txt).

    - If it is a [catkin metapackage](http://wiki.ros.org/catkin/package.xml#Metapackages) it must have the relevant boilerplate (= text that can be copied and used in legal documents or in computer programs, with only very small changes) CMakeLists.txt file.

- Each package must have its own folder

    - This means no nested packages nor multiple packages sharing the same directory.
  
The simplest possible package might have a structure which looks like this:

    my_package/
        CMakeLists.txt
        package.xml

## 4.2 Packages in a catkin Workspace

The recommended method of working with catkin packages is using a [catkin workspace](http://wiki.ros.org/catkin/workspaces), but you can also build catkin packages standalone. A trivial workspace might look like this:

    workspace_folder/        -- WORKSPACE
        src/                   -- SOURCE SPACE
        CMakeLists.txt       -- 'Toplevel' CMake file, provided by catkin
        package_1/
            CMakeLists.txt     -- CMakeLists.txt file for package_1
            package.xml        -- Package manifest for package_1
        ...
        package_n/
            CMakeLists.txt     -- CMakeLists.txt file for package_n
            package.xml        -- Package manifest for package_n


A catkin workspace is a folder where you modify, build, and install catkin packages. The following is the recommended and [typical catkin workspace](http://wiki.ros.org/catkin/workspaces#Catkin_Workspaces) layout:

    workspace_folder/         -- WORKSPACE
        src/                    -- SOURCE SPACE
        CMakeLists.txt        -- The 'toplevel' CMake file
        package_1/
            CMakeLists.txt
            package.xml
            ...
        package_n/
            CATKIN_IGNORE       -- Optional empty file to exclude package_n from being processed
            CMakeLists.txt
            package.xml
            ...
        build/                  -- BUILD SPACE
            CATKIN_IGNORE         -- Keeps catkin from walking this directory
        devel/                  -- DEVELOPMENT SPACE (set by CATKIN_DEVEL_PREFIX)
            bin/
            etc/
            include/
            lib/
            share/
            .catkin
            env.bash
            setup.bash
            setup.sh
            ...
        install/                -- INSTALL SPACE (set by CMAKE_INSTALL_PREFIX)
            bin/
            etc/
            include/
            lib/
            share/
            .catkin             
            env.bash
            setup.bash
            setup.sh
            ...


**Before continuing with this tutorial, make sure you have created an empty catkin workspace by following the [Creating a workspace for catkin tutorial](2_Create_a_ROS_Workspace.md).**

## 4.3 Packages in a catkin Workspace

This tutorial will demonstrate how to use the [catkin_create_pkg](http://wiki.ros.org/catkin/commands/catkin_create_pkg) script to create a new catkin package, and what you can do with it after it has been created.

First change to the source space directory of the catkin workspace you created in the [Creating a Workspace for catkin tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace):

    # You should have created this in the Creating a Workspace Tutorial
    cd ~/catkin_ws/src

Now use the *catkin_create_pkg* script to create a new package called 'beginner_tutorials' which depends on *std_msgs*, *roscpp*, and *rospy*:

    catkin_create_pkg beginner_tutorials std_msgs rospy roscpp

This will create a *beginner_tutorials* folder which contains a [package.xml](http://wiki.ros.org/catkin/package.xml) and a [CMakeLists.txt](http://wiki.ros.org/catkin/CMakeLists.txt), which have been partially filled out with the information you gave *catkin_create_pkg*.

*catkin_create_pkg* requires that you give it a *package_name* and optionally a list of dependencies on which that package depends:

    # This is an example, do not try to run this
    # catkin_create_pkg <package_name> [depend1] [depend2] [depend3]

*catkin_create_pkg* also has more advanced functionalities which are described in [catkin/commands/catkin_create_pkg](https://github.com/ros-infrastructure/catkin_pkg).

Now run this command (Replace `<workspace name>` with your name of workspace):

    cd ; cd ~/<workspace name>/src/beginner_tutorials ; ls

The output should look like this:

    CMakeLists.txt  include  package.xml  src

Open Files and navigate to ~/catkin_ws/src/beginner_tutorials. Take a look at the structure of  *CMakeLists.txt* file and compare with [CMakeLists.txt which uses catkin](http://wiki.ros.org/catkin/CMakeLists.txt).

## 4.4 Building a catkin workspace and sourcing the setup file

Now you need to build the packages in the catkin workspace:

    cd ~/catkin_ws
    catkin_make

Run:

    cd ~/catkin_ws/devel ; ls

The output should look like this:

    cmake.lock  local_setup.bash  setup.bash      setup.zsh
    env.sh      local_setup.sh    setup.sh        share
    lib         local_setup.zsh   _setup_util.py

To add the workspace to your ROS environment you need to source the generated setup file:

    . ~/catkin_ws/devel/setup.bash

## 4.5 Packages dependencies

### 4.5.1 First-order dependencies

When using [catkin_create_pkg](https://github.com/ros-infrastructure/catkin_pkg) earlier, a few package dependencies were provided. These **first-order** dependencies can now be reviewed with the *rospack* tool.

Run command:

    rospack depends1 beginner_tutorials 

Output:

    roscpp
    rospy
    std_msgs

As you can see, *rospack* lists the same dependencies that were used as arguments when running *catkin_create_pkg*. These dependencies for a package are stored in the **package.xml** file:

    roscd beginner_tutorials
    cat package.xml

Output:

    <?xml version="1.0"?>
    <package format="2">
    ...
        <buildtool_depend>catkin</buildtool_depend>
        <build_depend>roscpp</build_depend>
        <build_depend>rospy</build_depend>
        <build_depend>std_msgs</build_depend>
    ...
    </package>

### 4.5.2 Indirect dependencies

In many cases, a dependency will also have its own dependencies. For instance, rospy has other dependencies.

Run:

    rospack depends1 rospy

Output:

    genpy
    roscpp
    rosgraph
    rosgraph_msgs
    roslib
    std_msgs

A package can have quite a few indirect dependencies. Luckily rospack can **recursively** determine all nested dependencies.

Run:

    rospack depends beginner_tutorials

Output:

    cpp_common
    rostime
    roscpp_traits
    roscpp_serialization
    catkin
    genmsg
    genpy
    message_runtime
    gencpp
    geneus
    gennodejs
    genlisp
    message_generation
    rosbuild
    rosconsole
    std_msgs
    rosgraph_msgs
    xmlrpcpp
    roscpp
    rosgraph
    ros_environment
    rospack
    roslib
    rospy

## 4.6 Customizing Your Package

This part of the tutorial will look at each file generated by catkin_create_pkg and describe, line by line, each component of those files and how you can customize them for your package.

### 4.6.1 Customizing the package.xml

The generated [package.xml](http://wiki.ros.org/catkin/package.xml) should be in your new package. Now lets go through the new package.xml and touch up any elements that need your attention.

#### 4.6.1.1 description tag

First update the description tag:

    5   <description>The beginner_tutorials package</description>

Change the description to anything you like, but by convention the first sentence should be short while covering the scope of the package. If it is hard to describe the package in a single sentence then it might need to be broken up.

#### 4.6.1.2 maintainer tags

Next comes the maintainer tag:

     7   <!-- One maintainer tag required, multiple allowed, one person per tag --> 
     8   <!-- Example:  -->
     9   <!-- <maintainer email="jane.doe@example.com">Jane Doe</maintainer> -->
    10  <maintainer email="user@todo.todo">user</maintainer>

This is a required and important tag for the [package.xml](http://wiki.ros.org/catkin/package.xml) because it lets others know who to contact about the package. At least one maintainer is required, but you can have many if you like. The name of the maintainer goes into the body of the tag, but there is also an email attribute that should be filled out:

    7   <maintainer email="you@yourdomain.tld">Your Name</maintainer>

For example:

    7   <maintainer email="trung.lyduc18@gmail.com">Ly Duc Trung</maintainer>

#### 4.6.1.3 license tags

Next is the license tag, which is also required:

    12   <!-- One license tag required, multiple allowed, one license per tag -->
    13   <!-- Commonly used license strings: -->
    14   <!--   BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3 -->
    15   <license>TODO</license>

You should choose a license and fill it in here. Some common open source licenses are BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, and LGPLv3. You can read about several of these at the Open Source Initiative. For this tutorial we'll use the BSD license because the rest of the core ROS components use it already:





