# **Ubuntu install of ROS Noetic**

> NOTE: This instruction focuses on the perfomance installation and running commands. For more detail, you can read pdf book [ROS Robot Programming (35.6 MB)](https://www.robotis.com/service/download.php?no=719) and go to [ROS Official Tutorials](https://wiki.ros.org/ROS/Tutorials).

<br>

### Previous: [11. Build System #4: Build the System](11-Build-System-(4)-Build-the-System.md)

<br>

# 13. ROS Tools #1: 3D Visualization Tool (RViz)

RViz 1 is the 3D visualization tool of ROS. The main purpose is to show ROS messages in 3D, allowing us to visually verify data. For example, it can visualize the distance from the sensor of a Laser Distance Sensor (LDS) to an obstacle, the Point Cloud Data (PCD) of the 3D distance sensor such as RealSense, Kinect, or Xtion, the image value obtained from a camera, and many more without having to separately develop the software.

Loading screen of RViz, the 3D visualization tool of ROS:

![Loading screen of RViz](../Images/Loading_screen_of_RViz.png)

RViz example 1: Navigation using TurtleBot3 and LDS sensor

![RViz example 1](../Images/RViz_example_1.png)

RViz example 2: Obtain the skeleton of a person using Kinect and Command with a Motion

![RViz example 2](../Images/RViz_example_2.png)

RViz example 3: Measuring distance using LDS

![RViz example 3](../Images/RViz_example_3.png)

RViz example 4: distance, infrared, color image value obtained from Intel RealSense

![RViz example 4](../Images/RViz_example_4.png)

## 1. Installing and Running RViz

If you have installed ROS with ‘ros-[ROS_DISTRO]-desktop-full’ command, RViz should be installed by default. If you did not install the ‘desktop-full’ version ROS or for some reason, RViz is not installed, use the following command to install RViz.

    sudo apt-get install ros-kinetic-rviz

The execution command of RViz is as follows. However, just as for any other ROS tool, roscore must be running. For your reference, you can also run it with the node running command ‘rosrun rviz rviz’.

    rviz

## 2. RViz Screen Components

![Composition_of_the_RVi_ screen](../Images/Composition_of_the_RVi_screen.png)

**`(1)`** **3D View:** This black area is located in the middle of the screen. It is the main screen which allows us to see various data in 3D. Options such as background color of the 3D view, fixed frame, and grid can be configured in the Global Options and Grid settings on the left column of the screen.

**`(2)`** **Displays:** The Displays panel on the left column is for selecting the data that we want to display from the various topics. If we click `[Add]` button on the lower left corner of the panel, the display 4 selection screen will appear as shown in Figure 6-7. Currently, there are about 30 different types of displays we can choose from, which we will explore more in the following section.

**`(3)`** **Menu:** The Menu bar is located on the top of the screen. We can select commands to save or load the current display settings, and also can select various panels.

**`(4)`** **Tools:** Tools are located below the menu bar, where we can select buttons for various functions such as interact, camera movement, selection, camera focus change, distance measurement, 2D position estimation, 2D navigation target-point, publish point.

**`(5)`** **Views:** The Views panel configures the viewpoint of the 3D view.

- **Orbit:** The specified point is called the focus and the orbit rotates around this point. This is the default value and the most commonly used view.

- **FPS(first-person):** This displays in a first-person viewpoint.

- **ThirdPersonFollower:** This displays in a third-person viewpoint that follows a specific target.

- **TopDownOrtho:** This uses the Z-axis as the basis, and displays an orthographic projection of objects on the XY plane.

- XYOrbit: This is similar to the default setting which is the Orbit, but the focus is fixed on the XY plane, where the value of Z-axis coordinate is fixed to zero.

**`(6)`** **Time:** Time shows the current time (wall time), ROS Time, and the elapsed time for each of
them. This is mainly used in simulations, and if there is a need to restart it, simply click the
[Reset] button at the very bottom.

## 3. RViz Displays

![RViz display select screen](../Images/RViz_display_select_screen.png)


The most frequently used menu when using RViz will probably be the Displays 5 menu. This
Displays menu is used to select the message to display on the 3D View panel, and descriptions on each item are explained as follow.

| Name      | Description |
| ----------- | ----------- |
| Axes    | Displays the xyz axes. |
| Camera    | Creates a new rendering window from the camera perspective and overlays an image on top of it. |
| DepthCloud    | Displays a point cloud based on the Depth Map. It displays distance values acquired from sensors such as Kinect and Xtion with DepthMap and ColorImage topics as points with overlayed color obtained from the camera. |
| Effort    | Displays the force applied to each rotary joint of the robot. |
| FluidPressure    | Displays the pressure of fluids, such as air or water. |
| Grid    | Displays 2D or 3D grids. |
| Grid Cells    | Displays each cells of the grid. It is mainly used to display obstacles in the costmap of the navigation. |
| Illuminance    | Displays the illuminance. |
| Image    | Displays the image in a new rendering window. Unlike the Camera display, it does not overlay the camera. |
| InteractiveMarkers    | Displays Interactive Markers. We can change the position (x, y, z) and rotation (roll, pitch, yaw) with the mouse. |
| LaserScan    | Displays the laser scan value. |
| Map    | Displays the occupancy map, used in navigation, on top of the ground plane. |
| Marker    | Displays markers such as arrows, circles, triangles, rectangles, and cylinders provided by RViz. |
| MarkerArray    | Displays multiple markers. |
| Odometry    | Displays the odometry information in relation to the passage of time in the form of arrows. For example, as the robot moves, arrow markers are displayed showing the traveled path in a connected form according to the time intervals. |
| Path    | Displays the path of the robot used in navigation. |
| Point Cloud    | Displays point cloud data. This is used to display sensor data from depth cameras such as RealSense, Kinect, Xtion, etc. Since PointCloud2 is compatible with the latest Point Cloud Library (PCL), we can generally use PointCloud2. |
| Point Cloud2   |  as above |
| PointStamped    | Displays a rounded point. |
| Polygon    | Displays a polygon outline. It is mainly used to simply display the outline of a robot on the 2D plane. |
| Pose    | Displays the pose (location + orientation) on 3D. The pose is represented in the shape of an arrow where the origin of the arrow is the position (x, y, z,) and the direction of the arrow is the orientation (roll, pitch, yaw). For instance, pose can be represented with the position and orientation of the 3D robot model, while it can be represented with the goal point. |
| Pose Array    | Displays multiple poses. |
| Range    | This is used to visualize the measured range of a distance sensor such as an ultrasonic sensor or an infrared sensor in the form of a cone. |
| RelativeHumidity    | Displays the relative humidity. |
| RobotModel    | Displays the robot model. |
| TF    | Displays the coordinate transformation TF used in ROS. It is displayed with the xyz axes much like the previously mentioned axes, but each axis expresses the hierarchy with an arrow according to the relative coordinates. |
| Temperature    | Displays the temperature. |
| WrenchStamped    | Displays the wrench, which is the torsion movement, in the form of ‘arrow’ (force) and ‘arrow+circle’ (torque). |
