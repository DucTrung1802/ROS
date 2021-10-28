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

Now use the **catkin_create_pkg** script to create a new package called 'beginner_tutorials' which depends on **std_msgs**, **roscpp**, and **rospy**:

    catkin_create_pkg beginner_tutorials std_msgs rospy roscpp

This will create a **beginner_tutorials** folder which contains a [package.xml](http://wiki.ros.org/catkin/package.xml) and a [CMakeLists.txt](http://wiki.ros.org/catkin/CMakeLists.txt), which have been partially filled out with the information you gave *catkin_create_pkg*.

*catkin_create_pkg* requires that you give it a *package_name* and optionally a list of dependencies on which that package depends:

    # This is an example, do not try to run this
    # catkin_create_pkg <package_name> [depend1] [depend2] [depend3]

**catkin_create_pkg** also has more advanced functionalities which are described in [catkin/commands/catkin_create_pkg](https://github.com/ros-infrastructure/catkin_pkg).

Now run this command (Replace `<workspace name>` with your name of workspace):

    cd ; cd ~/<workspace name>/src/beginner_tutorials ; ls

The output should look like this:

    CMakeLists.txt  include  package.xml  src

Open Files and navigate to ~/catkin_ws/src/beginner_tutorials. Take a look at the structure of  **CMakeLists.txt** file and compare with [CMakeLists.txt which uses catkin](http://wiki.ros.org/catkin/CMakeLists.txt).

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

    cd
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

    cd
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

    cd
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

You should choose a license and fill it in here. Some common open source licenses are BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, and LGPLv3. You can read about several of these at the [Open Source Initiative](http://opensource.org/licenses/alphabetical). For this tutorial we'll use the BSD license because the rest of the core ROS components use it already:

    8   <license>BSD</license>

**NOTE:**

***[Apache License](https://en.wikipedia.org/wiki/Apache_License)*** = allows users to use the software for any purpose, to distribute it, to modify it, and to distribute modified versions of the software under the terms of the license, without concern for royalties (copyright fee). 

***[BSD Licenses](https://en.wikipedia.org/wiki/BSD_licenses)*** = Berkeley Software Distribution license = a family of permissive free software licenses, imposing minimal restrictions on the use and distribution of covered software.

***[GPL / GNU GPL](https://en.wikipedia.org/wiki/GNU_General_Public_License)*** = GNU General Public License (GNU GPL or simply GPL) = a series of widely used free software licenses that guarantee end users the freedom to run, study, share, and modify the software.

***[MIT License](https://vi.wikipedia.org/wiki/Gi%E1%BA%A5y_ph%C3%A9p_MIT)*** = a permissive free software license originating at the Massachusetts Institute of Technology (MIT) in the late 1980s. As a permissive license, it puts only very limited restriction on reuse and has, therefore, high license compatibility.

***[Mozilla License](https://www.mozilla.org/en-US/MPL/)*** = the custodian of the Mozilla Public License ("MPL"), an open source/free software license.

#### 4.6.1.4 dependencies tags

The next set of tags describe the dependencies of your package. The dependencies are split into **build_depend**, **buildtool_depend**, **exec_depend**, **test_depend**. For a more detailed explanation of these tags see the documentation about [Catkin Dependencies](). Since we passed **std_msgs**, **roscpp**, and **rospy** as arguments to [catkin_create_pkg](http://wiki.ros.org/catkin/commands/catkin_create_pkg), the dependencies will look like this:

    27   <!-- The *_depend tags are used to specify dependencies -->
    28   <!-- Dependencies can be catkin packages or system dependencies -->
    29   <!-- Examples: -->
    30   <!-- Use build_depend for packages you need at compile time: -->
    31   <!--   <build_depend>genmsg</build_depend> -->
    32   <!-- Use buildtool_depend for build tool packages: -->
    33   <!--   <buildtool_depend>catkin</buildtool_depend> -->
    34   <!-- Use exec_depend for packages you need at runtime: -->
    35   <!--   <exec_depend>python-yaml</exec_depend> -->
    36   <!-- Use test_depend for packages you need only for testing: -->
    37   <!--   <test_depend>gtest</test_depend> -->
    38   <buildtool_depend>catkin</buildtool_depend>
    39   <build_depend>roscpp</build_depend>
    40   <build_depend>rospy</build_depend>
    41   <build_depend>std_msgs</build_depend>

All of our listed dependencies have been added as a build_depend for us, in addition to the default buildtool_depend on catkin. In this case we want all of our specified dependencies to be **available at build and run time, so we'll add a exec_depend tag** for each of them as well:

    12   <buildtool_depend>catkin</buildtool_depend>
    13 
    14   <build_depend>roscpp</build_depend>
    15   <build_depend>rospy</build_depend>
    16   <build_depend>std_msgs</build_depend>
    17 
    18   <exec_depend>roscpp</exec_depend>
    19   <exec_depend>rospy</exec_depend>
    20   <exec_depend>std_msgs</exec_depend>

#### 4.6.1.5 Final package.xml

As you can see the final [package.xml](http://wiki.ros.org/catkin/package.xml), without comments and unused tags, is much more concise:

    1 <?xml version="1.0"?>
    2 <package format="2">
    3   <name>beginner_tutorials</name>
    4   <version>0.1.0</version>
    5   <description>The beginner_tutorials package</description>
    6 
    7   <maintainer email="you@yourdomain.tld">Your Name</maintainer>
    8   <license>BSD</license>
    9   <url type="website">http://wiki.ros.org/beginner_tutorials</url>
    10   <author email="you@yourdomain.tld">Jane Doe</author>
    11 
    12   <buildtool_depend>catkin</buildtool_depend>
    13 
    14   <build_depend>roscpp</build_depend>
    15   <build_depend>rospy</build_depend>
    16   <build_depend>std_msgs</build_depend>
    17 
    18   <exec_depend>roscpp</exec_depend>
    19   <exec_depend>rospy</exec_depend>
    20   <exec_depend>std_msgs</exec_depend>
    21 
    22 </package>

### 4.6.2 Customizing the CMakeLists.txt

Now that the [package.xml](http://wiki.ros.org/catkin/package.xml), which contains meta information, has been tailored to your package, you are ready to move on in the tutorials. The [CMakeLists.txt](http://wiki.ros.org/catkin/CMakeLists.txt) file created by [catkin_create_pkg](http://wiki.ros.org/catkin/commands/catkin_create_pkg) will be covered in the later tutorials about building ROS code.

Now that you've made a new ROS package, let's build our ROS package.

<br>

### Next: [5. Building a ROS Package](5_Building_a_ROS_Package.md)


