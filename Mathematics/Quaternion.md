# Quaternions

In part 4.5 Coordinate Transformation (TF) in [ROS Robot Programming (35.6 MB)](https://www.robotis.com/service/download.php?no=719) book, the word "**Quaternions**" was mentioned. Let's dig deeper to comprehend this mathematical term.

## 1. Definition

Quaternion is a Coordinate Transformation. It is a four-dimensional extension of a complex number. It describes a way to visualize a hypersphere using stereographic projection.

To easy for understanding, **Quaternions** is a complex number with 01 real part and 03 imaginary parts:

![Expression of Quaternion](https://s0.wp.com/latex.php?latex=%5Cboldsymbol%7Bq%7D+%3D+w+%2B+x%5Cboldsymbol%7Bi%7D+%2B+y%5Cboldsymbol%7Bj%7D+%2B+z%5Cboldsymbol%7Bk%7D+&bg=ffffff&fg=333333&s=1&c=20201002&zoom=2)

or

![Expression of Quaternion](https://s0.wp.com/latex.php?latex=%5Cboldsymbol%7Bq%7D+%3D+%5Ccos+%5Cfrac%7B1%7D%7B2%7D+%5Ctheta+%2B+%5Cboldsymbol%7Bu%7D+%5Csin+%5Cfrac%7B1%7D%7B2%7D+%5Ctheta+%3D+%5Ccos+%5Cfrac%7B1%7D%7B2%7D+%5Ctheta+%2B+%28u_x+%5Cboldsymbol%7Bi%7D+%2B+u_y+%5Cboldsymbol%7Bj%7D+%2B+u_z+%5Cboldsymbol%7Bk%7D%29+%5Csin+%5Cfrac%7B1%7D%7B2%7D+%5Ctheta+&bg=ffffff&fg=333333&s=1&c=20201002&zoom=2)

For more information, you can go to website: https://eater.net/quaternions

**Thanks to Grant Sanderson, Ben Eater and their team for contributions and publication a detail website about Quaternion.**


## 2. Role of Quaternion in ROS 

In ROS, the coordinate transformation TF is one of the most useful concepts when describing
the robot parts as well as obstacles and objects. The pose can be described as a combination of
positions and orientations. Here, the position is expressed by three vectors x, y, and z, and the
orientation by four vectors x, y, z, and w, called a quaternion. The quaternion is not intuitive
because they do not describe the rotation of three axes (x, y, z), such as the roll, pitch, and yaw
angles that are often used. However, the quaternion form is free from the gimbal lock or speed
issues that present in the Euler method of roll, pitch and yaw vectors. Therefore, the quaternion
type is preferred in robotics, and ROS also uses quaternion for this reason. Of course, functions
to convert Euler values to quaternions are provided for convenience.

At here, you may wonder why we choose Quaternion but not other type of coordinates.

## 3. Another Coordinate Transformation

### 3.1 Euler Angles



Ref: http://motion.pratt.duke.edu/RoboticSystems/3DRotations.html