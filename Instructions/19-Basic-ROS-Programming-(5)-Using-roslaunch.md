# **Ubuntu install of ROS Noetic**

> NOTE: This instruction focuses on the perfomance installation and running commands. For more detail, you can read pdf book [ROS Robot Programming (35.6 MB)](https://www.robotis.com/service/download.php?no=719) and go to [ROS Official Tutorials](https://wiki.ros.org/ROS/Tutorials).

<br>

### Previous: [17. Basic ROS Programming #3: Writing and Running the Action Server and Client Node](17-Basic-ROS-Programming-(3)-Writing-and-Running-the-Action-Server-and-Client-Node.md)

<br>

# 19. Basic ROS Programming #5: Using roslaunch

The ‘rosrun’ is a command that executes just one node, and ‘roslaunch’ can run more than one node. Other features of the ‘roslaunch’ command include the ability to modify parameters of the package, rename the node name, the ROS_ROOT and ROS_PACKAGE_PAPATION_PATH settings, and change environment variables.

The ‘roslaunch’ uses the ‘*.launch’ file to select executable nodes, which is XML-based and provides tag-specific options. The execution command is ‘roslaunch [package name] [roslaunch file]’.

## 1. Using the roslaunch

To learn how to use roslaunch, rename the ‘topic_publisher’ and ‘topic_subscriber’ nodes previously created. There is no point of only changing the names, so let’s run two sets of publisher and subscriber nodes to communicate with each other.

First, write a `‘*.launch’` file. The file used for roslaunch has a `‘*.launch’` extension file name, and you have to create a ‘launch’ folder in the package folder and place the launch file in that folder. Create a folder with the following command and create a new file called ‘union.launch’.

Run commands:

    roscd ros_tutorials_topic
    mkdir launch
    cd launch
    gedit union.launch

Write the contents of the ‘union.launch’ file as follows.

    <launch>
        <node pkg="ros_tutorials_topic" type="topic_publisher" name="topic_publisher1"/>
        <node pkg="ros_tutorials_topic" type="topic_subscriber" name="topic_subscriber1"/>
        <node pkg="ros_tutorials_topic" type="topic_publisher" name="topic_publisher2"/>
        <node pkg="ros_tutorials_topic" type="topic_subscriber" name="topic_subscriber2"/>
    </launch>

The tags required to run the node with the ‘roslaunch’ command are described within the `<launch>` tag. The `<node>` tag describes the node to be executed by ‘roslaunch’. Options include ‘pkg’, ‘type’, and ‘name’.

- **pkg:** Package name

- **type:** Name of the actual node to be executed (Node Name)

- **name:** The name (executable name) to used when the node corresponding to the ‘type’ above is executed. The name is generally set to be the same as the type, but it can be set to use a different name when executed.

Once the ‘roslaunch’ file is created, run ‘union.launch’ as follows. Note that when the ‘roslaunch’ command runs several nodes, the output (info, error, etc.) of the executed nodes is not displayed on the terminal screen, making it difficult to debug. If you add the ‘--screen’ option, the output of all nodes running on that terminal will be displayed on the terminal screen.

Run:

    roslaunch ros_tutorials_topic union.launch --screen

What would the screen look like if we run it? First, let’s take a look at the nodes currently running with the following command.

Open another terminal and run:

    rosnode list

Output:

    /rosout
    /topic_publisher1
    /topic_publisher2
    /topic_subscriber1
    /topic_subscriber2

As a result, the ‘topic_publisher’ node is renamed and executed as ‘topic_publisher1’ and ‘topic_publisher2’. The ‘topic_subscriber’ node is also renamed and executed as ‘topic_subscriber1’ and ‘topic_subscriber2’.

The problem is that unlike the initial intention to “run two publisher nodes and two subscriber nodes and make them communicate with their corresponding pairs”, we can see through ‘rqt_graph’ as below figure that each subscriber is receiving a topic from both publishers. This is because we simply changed the name of the node to be executed without changing the name of the message to be used. Let’s fix this problem with namespace tag in ‘roslaunch’.

![Diagram showing multiple node execution using roslaunch](../Images/Diagram_showing_multiple_node_execution_using_roslaunch.png)






