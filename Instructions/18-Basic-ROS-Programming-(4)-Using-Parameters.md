# **Ubuntu install of ROS Noetic**

> NOTE: This instruction focuses on the perfomance installation and running commands. For more detail, you can read pdf book [ROS Robot Programming (35.6 MB)](https://www.robotis.com/service/download.php?no=719) and go to [ROS Official Tutorials](https://wiki.ros.org/ROS/Tutorials).

<br>

### Previous: [17. Basic ROS Programming #3: Writing and Running the Action Server and Client Node](17-Basic-ROS-Programming-(3)-Writing-and-Running-the-Action-Server-and-Client-Node.md)

<br>

# 18. Basic ROS Programming #4: Using Parameters

The concept of a parameter has been described several times so far with list of the parameters. Therefore, in this section, let’s learn how to use parameters with hands-on practice. Refer to Section 7 for terminology of parameters, and Section 12 for the ‘rospram’ command.

## 1. Writing the Node using Parameters

Let’s modify the ‘service_server.cpp’ source in the service server and the client node created in Section 16 to use parameters to perform arithmetic operations, rather than just adding two values entered as service request. Modify the ‘service_server.cpp’ source in the following order.

Run commands:

    roscd ros_tutorials_service/src
    gedit service_server.cpp

<pre>
// ROS Default Header File
#include "ros/ros.h"
// SrvTutorial Service File Header (Automatically created after build)
#include "ros_tutorials_service/SrvTutorial.h"

#define PLUS                1 		 // Addition
#define MINUS               2 		 // Subtraction
#define MULTIPLICATION      3 		 // Multiplication
#define DIVISION            4 		 // Division

int g_operator = PLUS;

// The process below is performed if there is a service request
// The service request is declared as 'req', and the service response is declared as 'res'
bool calculation(ros_tutorials_service::SrvTutorial::Request &req,
ros_tutorials_service::SrvTutorial::Response &res)
{
    // The operator will be selected according to the parameter value and calculate 'a' and 'b',
    // which were received upon the service request.
    // The result is stored as the Response value.
    switch(g_operator)
    {
        case PLUS:
            res.result = req.a + req.b; break;
        case MINUS:
            res.result = req.a - req.b; break;
        case MULTIPLICATION:
            res.result = req.a * req.b; break;
        case DIVISION:
            if(req.b == 0)
            {
                res.result = 0;
                break;
            }
            else
            {
                res.result = req.a / req.b;
                break;
            }
        default:
            res.result = req.a + req.b;
            break;
    }

    // Displays the values of 'a' and 'b' used in the service request, and the 'result' value
    // corresponding to the service response.
    ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
    ROS_INFO("sending back response: [%ld]", (long int)res.result);
    return true;
}

int main(int argc, char **argv)   // Node Main Function
{
    ros::init(argc, argv, "service_server"); 		 // Initializes Node Name
    ros::NodeHandle nh; 				 // Node handle declaration

    <b>nh.setParam("calculation_method", PLUS);		 // Reset Parameter Settings</b>
    // Declare service server 'service_server' using the 'SrvTutorial' service file
    // in the 'ros_tutorials_service' package. The service name is 'ros_tutorial_srv' and
    // it is set to execute a 'calculation' function when a service is requested.
    ros::ServiceServer ros_tutorial_service_server = nh.advertiseService("ros_tutorial_srv", calculation);
    ROS_INFO("ready srv server!");
    ros::Rate r(10);		
    // 10hz
    while (1)
    {
        // Select the operator according to the value received from the parameter.
        <b>nh.getParam("calculation_method", g_operator);</b>
        ros::spinOnce(); // Callback function process routine
        r.sleep(); 		// Sleep for routine iteration
    }
    
    return 0;
}
</pre>

As most of the contents are similar to the previous examples, let’s just take a look at the additional parts needed to use parameters. In particular, the ‘setParam’ and ‘getParam’ methods in bold font are the most important parts when using parameters. As they are very simple methods, it can be easily understood by just reading its usage.

## 2. Setting Parameters

The following code sets the parameter of ‘calculation_method’ to ‘PLUS’. Since the word ‘PLUS’ is defined as ‘1’ in the code in previous part, the ‘calculation_method’ parameter becomes ‘1’ and the service response will add the received values from the service request.

    nh.setParam("calculation_method", PLUS);

Note that parameters can be set to integers, floats, boolean, string, dictionaries, list, and so on. For example, ‘1’ is an integer, ‘1.0’ is a float, ‘internetofthings’ is a string, ‘true’ is a boolean, ‘[1,2,3]’ is a list of integers, and ‘a: b, c: d’ is a dictionary.

## 3. Reading Parameters

The following gets the parameter value from ‘calculation_method’ and sets it as the value of ‘g_operator’. As a result, ‘g_operator’ from the code in part 1 checks the parameter value in every ‘0.1’ seconds to determine which operation to use on the values received through the service request.

    nh.getParam("calculation_method", g_operator);



## 4. Building and Running Nodes

Rebuild the service server node in the ‘ros_tutorials_service’ package with the following command.

Open a new terminal and run:

    cd ~/catkin_ws && catkin_make

When the build is done, run the ‘service_server’ node of the ‘ros_tutorials_service’ package with the following command.

**`Note: These following terminals run together.`**

Run:

    roscore

Open a new terminal and run:

    rosrun ros_tutorials_service service_server

Output:

    [ INFO] [1637075462.736561000]: ready srv server!

## 5. Displaying Parameter Lists

The ‘rosparam list’ command displays a list of parameters currently used in the ROS network. From the displayed list, ‘/calculation_method’ is the parameter we used.

Open a new terminal and run command:

    rosparam list

Output:

    /calculation_method
    /rosdistro
    /roslaunch/uris/host_localhost__35469
    /rosversion
    /run_id

## 6. Example of Using Parameters

Set the parameters according to the following command, and verify that the service processing has changed while requesting the same service each time.

Open a new terminal and run:

    rosparam set /calculation_method 1  #  Addition
    rosservice call /ros_tutorial_srv 10 5  # Input variables a and b for arithmetic operation

Output:

    result: 15

Run:

    rosparam set /calculation_method 2  # Substraction
    rosservice call /ros_tutorial_srv 10 5

Output:

    result: 5

Run:

    rosparam set /calculation_method 3  # Multiplication
    rosservice call /ros_tutorial_srv 10 5

Output:

    result: 50

Run:

    rosparam set /calculation_method 4  # Division
    rosservice call /ros_tutorial_srv 10 5

Output:

    result: 2

The ‘calculation_method’ parameter can be changed with the ‘rosparam set’ command. With the changed parameters, you can see the different result values with the same input of ‘rosservice call /ros_tutorial_srv 10 5’. As shown in above example, parameters in ROS can control the flow, setting, and processing of nodes from outside the node. It’s a very useful feature so familiarize yourself with this feature even if you do not need it right away.

<br>

### Next: [19. Basic ROS Programming #5: Using roslaunch](19-Basic-ROS-Programming-(5)-Using-roslaunch.md)