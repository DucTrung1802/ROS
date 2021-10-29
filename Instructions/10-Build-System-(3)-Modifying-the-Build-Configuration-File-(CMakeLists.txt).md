# **Ubuntu install of ROS Noetic**

> NOTE: This instruction focuses on the perfomance installation and running commands. For more detail, you can read pdf book [ROS Robot Programming (35.6 MB)](https://www.robotis.com/service/download.php?no=719) and go to [ROS Official Tutorials](https://wiki.ros.org/ROS/Tutorials).

<br>

### Previous: [9. Build System #2: Modifying the Package Configuration File (package.xml)](9-Build-System-(2)-Modifying-the-Package-Configuration-File-(package.xml).md.md)

<br>

# 10. Build System #3: Modifying the Build Configuration File (CMakeLists.txt)

Catkin, the build system for ROS, uses CMake and describes the build environment in the
‘CMakeLists.txt’ in the package folder. It configures the executable file creation, dependency
package priority build, link creation, and so on. 

**The original file without any modifications is
shown below.**

    cmake_minimum_required(VERSION 3.0.2)
    project(my_first_ros_pkg)

    ## Compile as C++11, supported in ROS Kinetic and newer
    # add_compile_options(-std=c++11)

    ## Find catkin macros and libraries
    ## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
    ## is used, also find other catkin packages
    find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    )

    ## System dependencies are found with CMake's conventions
    # find_package(Boost REQUIRED COMPONENTS system)


    ## Uncomment this if the package has a setup.py. This macro ensures
    ## modules and global scripts declared therein get installed
    ## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
    # catkin_python_setup()

    ################################################
    ## Declare ROS messages, services and actions ##
    ################################################

    ## To declare and build messages, services or actions from within this
    ## package, follow these steps:
    ## * Let MSG_DEP_SET be the set of packages whose message types you use in
    ##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
    ## * In the file package.xml:
    ##   * add a build_depend tag for "message_generation"
    ##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
    ##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
    ##     but can be declared for certainty nonetheless:
    ##     * add a exec_depend tag for "message_runtime"
    ## * In this file (CMakeLists.txt):
    ##   * add "message_generation" and every package in MSG_DEP_SET to
    ##     find_package(catkin REQUIRED COMPONENTS ...)
    ##   * add "message_runtime" and every package in MSG_DEP_SET to
    ##     catkin_package(CATKIN_DEPENDS ...)
    ##   * uncomment the add_*_files sections below as needed
    ##     and list every .msg/.srv/.action file to be processed
    ##   * uncomment the generate_messages entry below
    ##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

    ## Generate messages in the 'msg' folder
    # add_message_files(
    #   FILES
    #   Message1.msg
    #   Message2.msg
    # )

    ## Generate services in the 'srv' folder
    # add_service_files(
    #   FILES
    #   Service1.srv
    #   Service2.srv
    # )

    ## Generate actions in the 'action' folder
    # add_action_files(
    #   FILES
    #   Action1.action
    #   Action2.action
    # )

    ## Generate added messages and services with any dependencies listed here
    # generate_messages(
    #   DEPENDENCIES
    #   std_msgs
    # )

    ################################################
    ## Declare ROS dynamic reconfigure parameters ##
    ################################################

    ## To declare and build dynamic reconfigure parameters within this
    ## package, follow these steps:
    ## * In the file package.xml:
    ##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
    ## * In this file (CMakeLists.txt):
    ##   * add "dynamic_reconfigure" to
    ##     find_package(catkin REQUIRED COMPONENTS ...)
    ##   * uncomment the "generate_dynamic_reconfigure_options" section below
    ##     and list every .cfg file to be processed

    ## Generate dynamic reconfigure parameters in the 'cfg' folder
    # generate_dynamic_reconfigure_options(
    #   cfg/DynReconf1.cfg
    #   cfg/DynReconf2.cfg
    # )

    ###################################
    ## catkin specific configuration ##
    ###################################
    ## The catkin_package macro generates cmake config files for your package
    ## Declare things to be passed to dependent projects
    ## INCLUDE_DIRS: uncomment this if your package contains header files
    ## LIBRARIES: libraries you create in this project that dependent projects also need
    ## CATKIN_DEPENDS: catkin_packages dependent projects also need
    ## DEPENDS: system dependencies of this project that dependent projects also need
    catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES my_first_ros_pkg
    #  CATKIN_DEPENDS roscpp std_msgs
    #  DEPENDS system_lib
    )

    ###########
    ## Build ##
    ###########

    ## Specify additional locations of header files
    ## Your package locations should be listed before other locations
    include_directories(
    # include
    ${catkin_INCLUDE_DIRS}
    )

    ## Declare a C++ library
    # add_library(${PROJECT_NAME}
    #   src/${PROJECT_NAME}/my_first_ros_pkg.cpp
    # )

    ## Add cmake target dependencies of the library
    ## as an example, code may need to be generated before libraries
    ## either from message generation or dynamic reconfigure
    # add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

    ## Declare a C++ executable
    ## With catkin_make all packages are built within a single CMake context
    ## The recommended prefix ensures that target names across packages don't collide
    # add_executable(${PROJECT_NAME}_node src/my_first_ros_pkg_node.cpp)

    ## Rename C++ executable without prefix
    ## The above recommended prefix causes long target names, the following renames the
    ## target back to the shorter version for ease of user use
    ## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
    # set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

    ## Add cmake target dependencies of the executable
    ## same as for the library above
    # add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

    ## Specify libraries to link a library or executable target against
    # target_link_libraries(${PROJECT_NAME}_node
    #   ${catkin_LIBRARIES}
    # )

    #############
    ## Install ##
    #############

    # all install targets should use catkin DESTINATION variables
    # See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

    ## Mark executable scripts (Python etc.) for installation
    ## in contrast to setup.py, you can choose the destination
    # catkin_install_python(PROGRAMS
    #   scripts/my_python_script
    #   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    # )

    ## Mark executables for installation
    ## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
    # install(TARGETS ${PROJECT_NAME}_node
    #   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    # )

    ## Mark libraries for installation
    ## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
    # install(TARGETS ${PROJECT_NAME}
    #   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    #   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    #   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
    # )

    ## Mark cpp header files for installation
    # install(DIRECTORY include/${PROJECT_NAME}/
    #   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    #   FILES_MATCHING PATTERN "*.h"
    #   PATTERN ".svn" EXCLUDE
    # )

    ## Mark other files for installation (e.g. launch and bag files, etc.)
    # install(FILES
    #   # myfile1
    #   # myfile2
    #   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    # )

    #############
    ## Testing ##
    #############

    ## Add gtest based cpp test target and link libraries
    # catkin_add_gtest(${PROJECT_NAME}-test test/test_my_first_ros_pkg.cpp)
    # if(TARGET ${PROJECT_NAME}-test)
    #   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
    # endif()

    ## Add folders to be run by python nosetests
    # catkin_add_nosetests(test)

Options in the build configuration file (CMakeLists.txt) are as follows. The below describes **the minimum required version of ‘cmake’** installed on the operating system. Since it is currently specified as version 3.0.2, if you use a lower version of Cmake than this, you need to update the ‘cmake’ to meet the minimum requirement.

    cmake_minimum_required(VERSION 3.0.2)

The project describes **the name of the package**. Use the package name entered in ‘package. xml’. Note that if the package name is different from the package name described in the `<name>` tag in ‘package.xml’, an error will occur when building the package.

    project(my_first_ros_pkg)

The ‘find_package’ entry is the component package required to perform a build on Catkin. In this example, ‘roscpp’ and ‘std_msgs’ are set as dependent packages. If the package entered here is not found in the system, an error will occur when building the package. In other words, this is an **option to require the installation of dependent packages** for the custom package.

    find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
    )

The following is a method used when using packages other than ROS. For example, when
using Boost, the ‘system’ package must be installed beforehand. This feature is **an option that allows you to install dependent packages**.

    find_package(Boost REQUIRED COMPONENTS system)

The ‘catkin_python_setup()’ is **an option when using Python with ‘rospy’**. It invokes the Python installation process ‘setup.py’.

    catkin_python_setup()

‘add_message_files’ is **an option to add a message file**. The ‘FILES’ option will automatically generate a header file (*.h) by referring to the ‘.msg’ files in the ‘msg’ folder of the current package. In this example, message files Message1.msg and Message2.msg are used.

    add_message_files(
        FILES
        Message1.msg
        Message2.msg
    )

‘add_service_files’ is **an option to add a service file to use**. The ‘FILES’ option will refer to ‘.srv’ files in the ‘srv’ folder in the package. In this example, you have the option to use the service files Service1.srv and Service2.srv.

    add_service_files(
        FILES
        Service1.srv
        Service2.srv
    )

‘generate_messages’ is **an option to set dependent messages**. This example sets the
DEPENDENCIES option to use the ‘std_msgs’ message package.

    generate_messages(
    DEPENDENCIES
    std_msgs
    )

‘generate_dynamic_reconfigure_options’ **loads configuration files that are referred when using ‘dynamic_reconfigure’**.

    generate_dynamic_reconfigure_options(
        cfg/DynReconf1.cfg
        cfg/DynReconf2.cfg
    )

The following are **the options when performing a build on Catkin**. ‘INCLUDE_DIRS’ is a setting that specifies to use the header file in the ‘include’ folder, which is the internal folder of the package. ‘LIBRARIES’ is a setting used to specify the package library in the following configuration. ‘CATKIN_DEPENDS’ specifies dependent packages and in this example, the dependent packages are set to ‘roscpp’ and ‘std_msgs’. ‘DEPENDS’ is a setting that describes system-dependent packages.

    catkin_package(
        INCLUDE_DIRS include
        LIBRARIES my_first_ros_pkg
        CATKIN_DEPENDS roscpp std_msgs
        DEPENDS system_lib
    )

‘include_directories’ is **an option to specify folders to include**. In the example, `‘${catkin_INCLUDE_DIRS}’` is configured, which refers to the header file the ‘include’ folder in the package. **To specify an additional include folder, append it to the next line of** `‘${catkin_INCLUDE_DIRS}’`.

    include_directories(
    ${catkin_INCLUDE_DIRS}
    )

‘add_library’ **declares the library to be created after the build**. The following option will create ‘my_first_ros_pkg’ library from  my_first_ros_pkg.cpp’ file in the ‘src’ folder.

    add_library(my_first_ros_pkg
        src/${PROJECT_NAME}/my_first_ros_pkg.cpp
    )

‘add_dependencies’ is a command to perform certain tasks prior to the build process such as creating dependent messages or dynamic reconfigurations. The following options describe the creation of dependent messages and dynamic reconfiguration, which are the dependencies of the 'my_first_ros_pkg’ library.


In addition, the Install option used when creating the official distribution ROS package and the testing option used for the package test is provided.

The following is the modified build configuration file (CMakeLists.txt). Modify the file for your package. For more information on how to use the configuration file, please refer to the packages of TurtleBot3 and ROBOTIS OP3 published at ‘https://github.com/ROBOTIS-GIT’.

    cmake_minimum_required(VERSION 3.0.2)
    project(my_first_ros_pkg)
    find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
    catkin_package(CATKIN_DEPENDS roscpp std_msgs)
    include_directories(${catkin_INCLUDE_DIRS})
    add_executable(hello_world_node src/hello_world_node.cpp)
    target_link_libraries(hello_world_node ${catkin_LIBRARIES})

<br>

### Next: [11. Build System #4: Build the System](11-Build-System-(4)-Build-the-System.md)

