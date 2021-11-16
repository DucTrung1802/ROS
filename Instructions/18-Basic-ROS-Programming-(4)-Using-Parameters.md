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

The following gets the parameter value from ‘calculation_method’ and sets it as the value of ‘g_operator’. As a result, ‘g_operator’ from the code in section 7.5.1 checks the parameter value in every ‘0.1’ seconds to determine which operation to use on the values received through the service request.

    nh.getParam("calculation_method", g_operator);

## 4. Building and Running Nodes






















