# **Ubuntu install of ROS Noetic**

> NOTE: This instruction focuses on the perfomance installation and running commands. For more detail, you can read pdf book [ROS Robot Programming (35.6 MB)](https://www.robotis.com/service/download.php?no=719) and go to [ROS Official Tutorials](https://wiki.ros.org/ROS/Tutorials).

<br>

### Previous: [4. Create a package](4_Creating_a_ROS_Package.md)

<br>

# 5. Building a ROS Package

## 5.1 Building Packages

As long as all of the system dependencies of your package are installed, we can now build your new package.

**`Note: If you installed ROS using apt or some other package manager, you should already have all of your dependencies.`**

Before continuing remember to source your environment setup file if you have not already. On Ubuntu it would be something like this:

    source /opt/ros/%YOUR_ROS_DISTRO%/setup.bash
    source /opt/ros/noetic/setup.bash             # For Noetic for instance

### 5.1.1 Using catkin_make

[catkin_make](http://wiki.ros.org/catkin/commands/catkin_make) is a command line tool which adds some convenience to the standard catkin workflow. You can imagine that [catkin_make](http://wiki.ros.org/catkin/commands/catkin_make) combines the calls to cmake and make in the standard CMake workflow.

Usage:

    # In a catkin workspace
    catkin_make [make_targets] [-DCMAKE_VARIABLES=...]

The above command will build **any packages** located in **`~/catkin_ws/src`**$ cd ~/catkin_ws
$ cd src
$ catkin_init_workspace
$ cd ..
$ mkdir build
$ cd build
$ cmake ../src -DCMAKE_INSTALL_PREFIX=../install -DCATKIN_DEVEL_PREFIX=../devel
$ make**. The equivalent commands to do this manually would be:

    cd ~/catkin_ws
    cd src
    catkin_init_workspace
    cd ..
    mkdir build
    cd build
    cmake ../src -DCMAKE_INSTALL_PREFIX=../install -DCATKIN_DEVEL_PREFIX=../devel
    make



If you would like to build specific packages in the workspace, invoke the following in the root of your workspace:

    catkin_make -DCATKIN_WHITELIST_PACKAGES="package1;package2"

If you want to revert back to building all packages, do the following:

    catkin_make -DCATKIN_WHITELIST_PACKAGES=""

After running catkin_make, you should notice two new folders in the root of your catkin workspace: the **build** and **devel** folders. The **build** folder is where cmake and make are invoked, and the **devel** folder contains any generated files and targets, plus setup.*sh files so that you can use it like it is installed.

You can pass any arguments to catkin_make that you would normally pass to make and cmake. For example, you can invoke the install target like this.

Run:

    cd ~/catkin_ws
    catkin_make install

Which would be equivalent to calling make like this (don't run, just for understanding):

    cd ~/catkin_ws/build
    # If cmake hasn't already been called
    cmake ../src -DCMAKE_INSTALL_PREFIX=../install -DCATKIN_DEVEL_PREFIX=../devel
    make
    make install

Now there should be a fourth folder in the root of your workspace: **install**. This is a [Filesystem Hierarchy Standard](https://en.wikipedia.org/wiki/Filesystem_Hierarchy_Standard) (FHS) compliant installation of all of the packages in your catkin workspace. It contains setup.*sh files which can be sourced, allowing you to utilize the packages your built.

**Before continue, let's read about [CMake](https://cmake.org/overview/)**

CMake is an **extensible**, **open-source system** that **manages the build process** in an operating system and in a compiler-independent manner.

### 5.1.2 Building Your Package

**`If you are using this page to build your own code, please also take a look at the later tutorials (C++)/(Python) since you may need to modify CMakeLists.txt.`**

You should already have a [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) and a new catkin package called **beginner_tutorials** from the previous tutorial, [Creating a Package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage). Go into the catkin workspace if you are not already there and look in the src folder:

    cd ~/catkin_ws/
    ls src

Output:

    beginner_tutorials/  CMakeLists.txt

You should see that there is a folder called **beginner_tutorials** which you created with [catkin_create_pkg](https://github.com/ros-infrastructure/catkin_pkg) in the previous tutorial. We can now build that package using [catkin_make](http://wiki.ros.org/catkin/commands/catkin_make):

    catkin_make

Output:

    Base path: /home/user/catkin_ws
    Source space: /home/user/catkin_ws/src
    Build space: /home/user/catkin_ws/build
    Devel space: /home/user/catkin_ws/devel
    Install space: /home/user/catkin_ws/install
    ####
    #### Running command: "make cmake_check_build_system" in "/home/user/catkin_ws/build"
    ####
    ####
    #### Running command: "make -j8 -l8" in "/home/user/catkin_ws/build"
    ####

To check the built files, run:

    cd ~/<workspace name>/build; ls

Replace `<workspace name>` by your workspace name.

Output should look like this:

    atomic_configure    CATKIN_IGNORE        CTestConfiguration.ini  Makefile
    beginner_tutorials  catkin_make.cache    CTestCustom.cmake       test_results
    bin                 CMakeCache.txt       CTestTestfile.cmake
    catkin              CMakeFiles           gtest
    catkin_generated    cmake_install.cmake  install_manifest.txt


## 5.2 Quick Commands

- **cw**: Move to the predefined catkin workspace directory ‘~/catkin_ws’
- **cs**: Move to the directory ‘~/catkin_ws/src’ in the catkin workspace directory that contains source files
- **cm**: Move to the catkin workspace directory ‘~/catkin_ws’, and build ROS packages with
‘catkin_make’ command

Run:

    # Set ROS alias command
    alias cw='cd ~/catkin_ws'
    alias cs='cd ~/catkin_ws/src'
    alias cm='cd ~/catkin_ws && catkin_make'

<br>

### Next: [6. ROS Operation Test](6_ROS_Operation_Test.md)