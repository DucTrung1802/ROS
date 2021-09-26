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

```shell
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

### 1.3 Set up your keys
Run the command:
```shell
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

### 1.4 Installation
First, make sure your Debian package index is up-to-date:
```shell
sudo apt update
```
Now pick how much of ROS you would like to install.

**Option 1 - Desktop-Full Install (Recommend):** Everything in Desktop plus 2D/3D simulators and 2D/3D perception packages
    
    sudo apt install ros-noetic-desktop-full




<!-- ### [Install ROS](https://wiki.ros.org/noetic/Installation/Ubuntu)



```shell
alo 123
```


**** -->



