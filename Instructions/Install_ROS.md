# **Ubuntu install of ROS Noetic**

## 1. Installation

### 1.1 Configure your Ubuntu repositories
- Open "Software & Updates" (purple color).
- In the tab "Ubuntu Software, check all the (main), (universe), (restricted), (multiverse).
- For the part "Download from", choose Main Server 
- **(IMPORTANT)** Check the "Cdrom with Ubuntu 20.04 'Focal Fossa'" (the version and name might change at the moment of installation)

### 1.2 Setup your sources.list
Setup your computer to accept software from `packages.ros.org`.

Open Terminal and run the command:

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'


### 1.3 Set up your keys
Run the command:

    sudo apt install curl # if you haven't already installed curl
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

### 1.4 Installation
First, make sure your Debian package index is up-to-date:

    sudo apt update

Now pick how much of ROS you would like to install.

**Option 1 - Desktop-Full Install (Recommend):** Everything in **Option 2** plus 2D/3D simulators and 2D/3D perception packages
    
    sudo apt install ros-noetic-desktop-full

**Option 2 - Desktop Install:** Everything in **Option 3** plus tools like rqt [rqt](https://wiki.ros.org/rqt) and rviz [rviz](https://wiki.ros.org/rviz)

**Option 3 - ROS-Base: (Bare Bones)** ROS packaging, build, and communication libraries. No GUI tools.

    sudo apt install ros-noetic-ros-base

### 1.5 Dependencies for building packages
To install this tool and other dependencies for building ROS packages, run:

    sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

#### 1.5.1 Initialize rosdep
Before you can use many ROS tools, you will need to initialize rosdep. rosdep enables you to easily install system dependencies for source you want to compile and is required to run some core components in ROS. If you have not yet installed rosdep, do so as follows.

    sudo apt install python3-rosdep

With the following, you can initialize rosdep.

    sudo rosdep init
    rosdep update

### 1.6 Checking the installation
If you are ever having problems finding or using your ROS packages make sure that you have your environment properly setup. A good way to check is to ensure that environment variables like ROS_ROOT and ROS_PACKAGE_PATH are set:

    printenv | grep ROS

If Terminal prints:

    ROS_VERSION=1
    ROS_PYTHON_VERSION=3
    ROS_PACKAGE_PATH=/opt/ros/noetic/share
    ROSLISP_PACKAGE_DIRECTORIES=
    ROS_ETC_DIR=/opt/ros/noetic/etc/ros
    ROS_MASTER_URI=http://localhost:11311
    ROS_ROOT=/opt/ros/noetic/share/ros
    ROS_DISTRO=noetic

Then you have successfully installed ROS and the environment.

## 2. Create a ROS Workspace

> These instructions are for ROS Groovy and later. For ROS Fuerte and earlier, select rosbuild.

### 2.1 Create workspace folder

Let's create and build a catkin workspace:

    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/
    catkin_make

> Your workspace name is up to you. In this case, my workspace name is "catkin_ws"

Output:

    Base path: /home/<your username>/catkin_ws
    Source space: /home/<your username>/catkin_ws/src
    Build space: /home/<your username>/catkin_ws/build
    Devel space: /home/<your username>/catkin_ws/devel
    Install space: /home/<your username>/catkin_ws/install
    Creating symlink "/home/<your username>/catkin_ws/src/CMakeLists.txt" pointing to "/opt/ros/noetic/share/catkin/cmake/toplevel.cmake"
    ####
    #### Running command: "cmake /home/<your username>/catkin_ws/src -DCATKIN_DEVEL_PREFIX=/home/<your username>/catkin_ws/devel -DCMAKE_INSTALL_PREFIX=/home/<your username>/catkin_ws/install -G Unix Makefiles" in "/home/<your username>/catkin_ws/build"
    ####
    -- The C compiler identification is GNU 9.3.0
    -- The CXX compiler identification is GNU 9.3.0
    -- Check for working C compiler: /usr/bin/cc
    -- Check for working C compiler: /usr/bin/cc -- works
    -- Detecting C compiler ABI info
    -- Detecting C compiler ABI info - done
    -- Detecting C compile features
    -- Detecting C compile features - done
    -- Check for working CXX compiler: /usr/bin/c++
    -- Check for working CXX compiler: /usr/bin/c++ -- works
    -- Detecting CXX compiler ABI info
    -- Detecting CXX compiler ABI info - done
    -- Detecting CXX compile features
    -- Detecting CXX compile features - done
    -- Using CATKIN_DEVEL_PREFIX: /home/<your username>/catkin_ws/devel
    -- Using CMAKE_PREFIX_PATH: /opt/ros/noetic
    -- This workspace overlays: /opt/ros/noetic
    -- Found PythonInterp: /usr/bin/python3 (found suitable version "3.8.10", minimum required is "3") 
    -- Using PYTHON_EXECUTABLE: /usr/bin/python3
    -- Using Debian Python package layout
    -- Found PY_em: /usr/lib/python3/dist-packages/em.py  
    -- Using empy: /usr/lib/python3/dist-packages/em.py
    -- Using CATKIN_ENABLE_TESTING: ON
    -- Call enable_testing()
    -- Using CATKIN_TEST_RESULTS_DIR: /home/<your username>/catkin_ws/build/test_results
    -- Forcing gtest/gmock from source, though one was otherwise available.
    -- Found gtest sources under '/usr/src/googletest': gtests will be built
    -- Found gmock sources under '/usr/src/googletest': gmock will be built
    -- Found PythonInterp: /usr/bin/python3 (found version "3.8.10") 
    -- Found Threads: TRUE  
    -- Using Python nosetests: /usr/bin/nosetests3
    -- catkin 0.8.10
    -- BUILD_SHARED_LIBS is on
    -- BUILD_SHARED_LIBS is on
    -- Configuring done
    -- Generating done
    -- Build files have been written to: /home/<your username>/catkin_ws/build
    ####
    #### Running command: "make -j8 -l8" in "/home/<your username>/catkin_ws/build"
    ####

Now, your current folder should have 3 folders as following. Run command:

    ls

Output:

    build  devel  src

### 2.2 Setup Environment

Each time you run a new Terminal for ROS, you need to include `source /opt/ros/noetic/setup.bash` and `source ~/FAIlearning_ws/devel/setup.bash`. For more convenient, we will automatically setup the ROS environment. 

Run the following command to open bashrc file:

    cd ~/<your workspace name>/
    gedit ~/.bashrc

Then, add this script and then save the file:

    source ~/<your workspace name>/devel/setup.bash

> If your workspace name is `catkin_ws`, the script will be `source ~/catkin_ws/devel/setup.bash`.











