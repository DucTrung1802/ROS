# **Ubuntu install of ROS Noetic**

> NOTE: This instruction focuses on the perfomance installation and running commands. For more detail, you can read pdf book [ROS Robot Programming (35.6 MB)](https://www.robotis.com/service/download.php?no=719) and go to [ROS Official Tutorials](https://wiki.ros.org/ROS/Tutorials).

# 1. Installation

## 1.1 Configure your Ubuntu repositories
- Open "Software & Updates" (purple color).
- In the tab "Ubuntu Software, check all the (main), (universe), (restricted), (multiverse).
- For the part "Download from", choose Main Server 
- **(IMPORTANT)** Check the "Cdrom with Ubuntu 20.04 'Focal Fossa'" (the version and name might change at the moment of installation)

## 1.2 Setup your sources.list
Setup your computer to accept software from `packages.ros.org`.

Open Terminal and run the command:

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'


## 1.3 Set up your keys
Run the command:

    sudo apt install curl # if you haven't already installed curl
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

## 1.4 Installation
First, make sure your Debian package index is up-to-date:

    sudo apt update

Now pick how much of ROS you would like to install.

**Option 1 - Desktop-Full Install (Recommend):** Everything in **Option 2** plus 2D/3D simulators and 2D/3D perception packages
    
    sudo apt install ros-noetic-desktop-full

**Option 2 - Desktop Install:** Everything in **Option 3** plus tools like rqt [rqt](https://wiki.ros.org/rqt) and rviz [rviz](https://wiki.ros.org/rviz)

**Option 3 - ROS-Base: (Bare Bones)** ROS packaging, build, and communication libraries. No GUI tools.

    sudo apt install ros-noetic-ros-base

## 1.5 Dependencies for building packages
To install this tool and other dependencies for building ROS packages, run:

    sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

### 1.5.1 Initialize rosdep
Before you can use many ROS tools, you will need to initialize rosdep. rosdep enables you to easily install system dependencies for source you want to compile and is required to run some core components in ROS. If you have not yet installed rosdep, do so as follows.

    sudo apt install python3-rosdep

With the following, you can initialize rosdep.

    sudo rosdep init
    rosdep update

## 1.6 Checking the installation
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

### Next: [2. Create a ROS Workspace](2_Create_a_ROS_Workspace.md)