# **Ubuntu install of ROS Noetic**

> NOTE: This instruction focuses on the perfomance installation and running commands. For more detail, you can read pdf book [ROS Robot Programming (35.6 MB)](https://www.robotis.com/service/download.php?no=719) and go to [ROS Official Tutorials](https://wiki.ros.org/ROS/Tutorials).

<br>

### Previous: [11. Build System #4: Build the System](11-Build-System-(4)-Build-the-System.md)

<br>

# 12. ROS Commands

This section is completely presented in Chapter 5 in [ROS Robot Programming (35.6 MB)](https://www.robotis.com/service/download.php?no=719), so I will not represent it here.

## Note

This section is for the actual change from the book [ROS Robot Programming (35.6 MB)](https://www.robotis.com/service/download.php?no=719).

### 5.4.5. rosparam: ROS Parameter

In part **`rosparam get [PARAMETER_NAME]: Get parameter value`**, if you run:

    rosparam get /background_b

Output will be an error:

    ERROR: Parameter [/background_b] is not set

This error occurs because the book was published in 2017 and the ROS Distribution has been continously updated over 5 years. You should **write** this command in terminal:

    rosparam get /

then press `TAB` two times to see the available parameters for the 'rosparam get' command.

Output should be:

    /rosdistro
    /roslaunch/uris/<host name>__<port>
    /rosversion
    /run_id
    /turtlesim/background_b
    /turtlesim/background_g
    /turtlesim/background_r

### 5.5. ROS Catkin Commands

In part **`catkin_eclipse: Modify package created by catkin build system so that it can
be used in Eclipse`**, if you run:

    cd ~/catkin_ws
    catkin_eclipse

Output will be an error:

    catkin_eclipse: command not found

The might be a change in ROS Distribution. Eclipse is just an Integrated Development Environment (IDE) so don't hesitate to use other IDE such as: Visual Studio Code, PyCharm, QtCreator and so on. I prefer to use Visual Studio Code since it is a lightweight and versatile software.

