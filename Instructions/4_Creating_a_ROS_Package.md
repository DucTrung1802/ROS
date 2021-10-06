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



