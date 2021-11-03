# **Ubuntu install of ROS Noetic**

> NOTE: This instruction focuses on the perfomance installation and running commands. For more detail, you can read pdf book [ROS Robot Programming (35.6 MB)](https://www.robotis.com/service/download.php?no=719) and go to [ROS Official Tutorials](https://wiki.ros.org/ROS/Tutorials).

<br>

### Previous: [10. Build System #3: Modifying the Build Configuration File (CMakeLists.txt)](10-Build-System-(3)-Modifying-the-Build-Configuration-File-(CMakeLists.txt).md)

<br>

# 11. Build System #4: Build the System

## 11.1 Writing Source Code

The following setting is configured in the executable file creation section (add_executable) of the ‘CMakeLists.txt’ file mentioned above.

Open terminal and run commands:

    cd ~/catkin_ws/src/my_first_ros_pkg
    gedit CMakeLists.txt

Replace all the existing content by the following soure code:

    cmake_minimum_required(VERSION 3.0.2)
    project(my_first_ros_pkg)
    find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
    catkin_package(CATKIN_DEPENDS roscpp std_msgs)
    include_directories(${catkin_INCLUDE_DIRS})
    add_executable(hello_world_node src/hello_world_node.cpp)
    target_link_libraries(hello_world_node ${catkin_LIBRARIES})

This is the setting to create the executable ‘hello_world_node’ by referring to the ‘hello_world_node’ source code in the ‘src’ folder of the package. As ‘hello_world_node.cpp’ source code has to be manually created and written by developer, let’s write a simple example.

First, move to the source code folder (src) in your package folder by using ‘cd’ command and create the ‘hello_world_node.cpp’ file as shown below. This example uses the gedit editor, but you can use your preferred editor, such as vi, gedit, qtcreator, vim, or emacs.

    cd ~/catkin_ws/src/my_first_ros_pkg/src/
    gedit hello_world_node.cpp

Then, write the following source code in the created file.

    #include <ros/ros.h>
    #include <std_msgs/String.h>
    #include <sstream>

    int main(int argc, char **argv)
    {
        ros::init(argc, argv, "hello_world_node");
        ros::NodeHandle nh;
        ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("say_hello_world", 1000);
        ros::Rate loop_rate(10);
        int count = 0;

        while (ros::ok())
        {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "hello world!" << count;
        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());
        chatter_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
        }

        return 0;
    }   

## 11.2 Building the Package

Once above code is saved in the file, all the necessary work for building a package is completed. Before building the package, update the profile of the ROS package with the below command. It is a command to apply the previously created package to the ROS package list. **Although this is not mandatory, it is convenient to update after creating a new package as it will allows to find the package using auto-completion feature with the Tab key**.

    rospack profile

The following is a Catkin build. Go to the Catkin workspace and build the package.

    cd ~/catkin_ws && catkin_make

### Alias Command

As mentioned in Section 5.2 Quick Commands, if you set alias cm=’cd ~/catkin_ws && catkin_make’ in the ‘.bashrc’ file, you can replace the above command with ‘cm’ command in the terminal window. As it is very useful, make sure to set it by referring to the ROS Development Environment setup section.

## 11.3 Running the Node

If the build was completed without any errors, a ‘hello_world_node’ file should have been created in ‘~/catkin_ws/devel/lib/my_first_ros_pkg’.

The next step is to run the node. Open a terminal window (Ctrl + Alt + t) and run roscore before running the node. Note that roscore must be running in order to execute ROS nodes, and roscore only needs to be run once unless it stops.

    roscore

Finally, open a new terminal window (Ctrl + Alt + t) and run the node with the command below. This is a command to run a node called ‘hello_world_node’ in a package named my_first_ros_pkg’.

    rosrun my_first_ros_pkg hello_world_node

Output:

    [ INFO] [1635531908.503232126]: hello world!0
    [ INFO] [1635531908.603528094]: hello world!1
    [ INFO] [1635531908.703408777]: hello world!2
    [ INFO] [1635531908.803699579]: hello world!3
    [ INFO] [1635531908.903855764]: hello world!4
    [ INFO] [1635531909.003606583]: hello world!5
    [ INFO] [1635531909.103781009]: hello world!6
    [ INFO] [1635531909.203768494]: hello world!7
    [ INFO] [1635531909.303646347]: hello world!8
    [ INFO] [1635531909.403631314]: hello world!9
    [ INFO] [1635531909.503624985]: hello world!10
    ...

<br>

### Next: [12. ROS Commands](12-ROS-Commands.md)