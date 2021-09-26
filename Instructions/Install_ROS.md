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

#### 1.5.1
Before you can use many ROS tools, you will need to initialize rosdep. rosdep enables you to easily install system dependencies for source you want to compile and is required to run some core components in ROS. If you have not yet installed rosdep, do so as follows.

    sudo apt install python3-rosdep

With the following, you can initialize rosdep.

    sudo rosdep init
    rosdep update



<!-- ### [Install ROS](https://wiki.ros.org/noetic/Installation/Ubuntu)



```shell
alo 123
```


**** -->



