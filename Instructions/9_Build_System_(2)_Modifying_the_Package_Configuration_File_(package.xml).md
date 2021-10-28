# **Ubuntu install of ROS Noetic**

> NOTE: This instruction focuses on the perfomance installation and running commands. For more detail, you can read pdf book [ROS Robot Programming (35.6 MB)](https://www.robotis.com/service/download.php?no=719) and go to [ROS Official Tutorials](https://wiki.ros.org/ROS/Tutorials).

<br>

### Previous: [8. Build System #1: Create your own ROS package](8_Build_System_(1)_Create_your_own_ROS_package.md)

<br>

# 9. Build System #2: Modifying the Package Configuration File (package.xml)

‘Package.xml’, which is one of the essential ROS configuration files, is an XML file containing
information about the package, including the package name, author, license, and dependent
packages. The original file without any modifications is shown below.

    <package format="2">
        <name>my_first_ros_pkg</name>
        <version>0.0.0</version>
        <description>The my_first_ros_pkg package</description>
        <!--  One maintainer tag required, multiple allowed, one person per tag  -->
        <!--  Example:   -->
        <!--  <maintainer email="jane.doe@example.com">Jane Doe</maintainer>  -->
        <maintainer email="<user name>@todo.todo"><user name></maintainer>
        <!--  One license tag required, multiple allowed, one license per tag  -->
        <!--  Commonly used license strings:  -->
        <!--    BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3  -->
        <license>TODO</license>
        <!--  Url tags are optional, but multiple are allowed, one per tag  -->
        <!--  Optional attribute type can be: website, bugtracker, or repository  -->
        <!--  Example:  -->
        <!--  <url type="website">http://wiki.ros.org/my_first_ros_pkg</url>  -->
        <!--  Author tags are optional, multiple are allowed, one per tag  -->
        <!--  Authors do not have to be maintainers, but could be  -->
        <!--  Example:  -->
        <!--  <author email="jane.doe@example.com">Jane Doe</author>  -->
        <!--  The *depend tags are used to specify dependencies  -->
        <!--  Dependencies can be catkin packages or system dependencies  -->
        <!--  Examples:  -->
        <!--  Use depend as a shortcut for packages that are both build and exec dependencies  -->
        <!--    <depend>roscpp</depend>  -->
        <!--    Note that this is equivalent to the following:  -->
        <!--    <build_depend>roscpp</build_depend>  -->
        <!--    <exec_depend>roscpp</exec_depend>  -->
        <!--  Use build_depend for packages you need at compile time:  -->
        <!--    <build_depend>message_generation</build_depend>  -->
        <!--  Use build_export_depend for packages you need in order to build against this package:  -->
        <!--    <build_export_depend>message_generation</build_export_depend>  -->
        <!--  Use buildtool_depend for build tool packages:  -->
        <!--    <buildtool_depend>catkin</buildtool_depend>  -->
        <!--  Use exec_depend for packages you need at runtime:  -->
        <!--    <exec_depend>message_runtime</exec_depend>  -->
        <!--  Use test_depend for packages you need only for testing:  -->
        <!--    <test_depend>gtest</test_depend>  -->
        <!--  Use doc_depend for packages you need only for building documentation:  -->
        <!--    <doc_depend>doxygen</doc_depend>  -->
        <buildtool_depend>catkin</buildtool_depend>
        <build_depend>roscpp</build_depend>
        <build_depend>std_msgs</build_depend>
        <build_export_depend>roscpp</build_export_depend>
        <build_export_depend>std_msgs</build_export_depend>
        <exec_depend>roscpp</exec_depend>
        <exec_depend>std_msgs</exec_depend>
        <!--  The export tag contains other, unspecified, tags  -->
        <export>
        <!--  Other tools can request additional information be placed here  -->
        </export>
    </package>

Below are descriptions of each statement.

| Syntax      | Description |
| ----------- | ----------- |
| `<?xml>`     | This tag indicates that the contents in the document abide by the XML Version 1.0. |
| `<package>`   | This tag is paired with </package> tag to indicate the configuration part of the ROS package configuration part.        |
| `<name>`   | This tag indicates the package name. The package name entered when creating the package is used. The name of the package can be changed by the developer.        |
| `<version>`   | This tag indicates the package version. The developer can assign the version of the package.        |
| `<description>`   | A short  escription of the package. Usually  -3 sentences.        |
| `<maintainer>`   | The name and email address of the package  dministrator.        |
| `<license>`   | This tag indicates the license, such as BSD, MIT,  pache, GPLv3, LGPLv3.        |
| `<url>`   | This tag indicates  ddress of the webpage describing the package, or bug management, repository, etc. Depending on the type, you can assign it as a website, bugtracker, or repository.        |
| `<author>`   | The name and email address of the developer who  articipated in the package development. If multiple developers were involved, append multiple <author> tags to the following lines.        |
| `<buildtool_depend>`   | Describes the dependencies of the build system. As we are using the Catkin build system, write ‘catkin’.        |
| `<build_depend>`   | Dependent  ackage name when building the  ackage.        |
| `<run_depend>`   | Dependent  ackage name when running the  ackage.        |
| `<test_depend>`   | Dependent  ackage name when testing the  ackage.        |
| `<export>`   | It is used when using a tag name that is not  pecified in ROS. The most widely used case is for metapackages. In this  ase, use `<export><metapackage/></export>` to notify that the package is a metapackage.        |
| `<metapackage>`   | The official tag used within the export tag that declares the current package as a metapackage.        |

I modified the package configuration file (package.xml) as follows. Let’s modify it in your own environment as well. If you are unfamiliar with it, you can use the below file as is:

    <?xml version="1.0"?>
    <package>
        <name>my_first_ros_pkg</name>
        <version>0.0.1</version>
        <description>The my_first_ros_pkg package</description>
        <license>Apache License 2.0</license>
        <author email="trung.lyduc18@gmail.com">Ly Duc Trung</author>
        <maintainer email="trung.lyduc18@gmail.com">Ly Duc Trung</maintainer>
        <url type="bugtracker"> [fill issues link git hub here] </url>
        <url type="repository"> [fill link of repo] </url>
        <url type="website"> [fill website] </url>
        <buildtool_depend>catkin</buildtool_depend>
        <build_depend>std_msgs</build_depend>
        <build_depend>roscpp</build_depend>
        <run_depend>std_msgs</run_depend>
        <run_depend>roscpp</run_depend>
        <export></export>
    </package>

### Next: [10. Build System (3): Modifying the Build Configuration File (CMakeLists.txt)](10_Build_System_(3)_Modifying_the_Build_Configuration_File(CMakeLists.txt).md)




