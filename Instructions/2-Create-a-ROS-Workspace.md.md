# **Ubuntu install of ROS Noetic**

> NOTE: This instruction focuses on the perfomance installation and running commands. For more detail, you can read pdf book [ROS Robot Programming (35.6 MB)](https://www.robotis.com/service/download.php?no=719) and go to [ROS Official Tutorials](https://wiki.ros.org/ROS/Tutorials).

<br>

### Previous: [1. Installation](1-Installation.md)

<br>

# 2. Create a ROS Workspace


> These instructions are for ROS Groovy and later. For ROS Fuerte and earlier, select rosbuild.

## 2.1 Create workspace folder

Let's create and build a catkin workspace:

    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/
    catkin_make

> Your workspace name is up to you. In this case, my workspace name is "catkin_ws"

Output:

    Base path: /home/<your username>/catkin_ws
    Source space: /home/<your username>/catkin_ws/src
    Build space: /home/<your username>/catkin_ws/build
    Devel space: /home/<your username>/catkin_ws/devel
    Install space: /home/<your username>/catkin_ws/install
    Creating symlink "/home/<your username>/catkin_ws/src/CMakeLists.txt" pointing to "/opt/ros/noetic/share/catkin/cmake/toplevel.cmake"
    ####
    #### Running command: "cmake /home/<your username>/catkin_ws/src -DCATKIN_DEVEL_PREFIX=/home/<your username>/catkin_ws/devel -DCMAKE_INSTALL_PREFIX=/home/<your username>/catkin_ws/install -G Unix Makefiles" in "/home/<your username>/catkin_ws/build"
    ####
    -- The C compiler identification is GNU 9.3.0
    -- The CXX compiler identification is GNU 9.3.0
    -- Check for working C compiler: /usr/bin/cc
    -- Check for working C compiler: /usr/bin/cc -- works
    -- Detecting C compiler ABI info
    -- Detecting C compiler ABI info - done
    -- Detecting C compile features
    -- Detecting C compile features - done
    -- Check for working CXX compiler: /usr/bin/c++
    -- Check for working CXX compiler: /usr/bin/c++ -- works
    -- Detecting CXX compiler ABI info
    -- Detecting CXX compiler ABI info - done
    -- Detecting CXX compile features
    -- Detecting CXX compile features - done
    -- Using CATKIN_DEVEL_PREFIX: /home/<your username>/catkin_ws/devel
    -- Using CMAKE_PREFIX_PATH: /opt/ros/noetic
    -- This workspace overlays: /opt/ros/noetic
    -- Found PythonInterp: /usr/bin/python3 (found suitable version "3.8.10", minimum required is "3") 
    -- Using PYTHON_EXECUTABLE: /usr/bin/python3
    -- Using Debian Python package layout
    -- Found PY_em: /usr/lib/python3/dist-packages/em.py  
    -- Using empy: /usr/lib/python3/dist-packages/em.py
    -- Using CATKIN_ENABLE_TESTING: ON
    -- Call enable_testing()
    -- Using CATKIN_TEST_RESULTS_DIR: /home/<your username>/catkin_ws/build/test_results
    -- Forcing gtest/gmock from source, though one was otherwise available.
    -- Found gtest sources under '/usr/src/googletest': gtests will be built
    -- Found gmock sources under '/usr/src/googletest': gmock will be built
    -- Found PythonInterp: /usr/bin/python3 (found version "3.8.10") 
    -- Found Threads: TRUE  
    -- Using Python nosetests: /usr/bin/nosetests3
    -- catkin 0.8.10
    -- BUILD_SHARED_LIBS is on
    -- BUILD_SHARED_LIBS is on
    -- Configuring done
    -- Generating done
    -- Build files have been written to: /home/<your username>/catkin_ws/build
    ####
    #### Running command: "make -j8 -l8" in "/home/<your username>/catkin_ws/build"
    ####

Now, your current folder should have 3 folders as following. Run command:

    ls

Output:

    build  devel  src

<br>

### Next: [3. Navigating the ROS Filesystem](3-Navigating-the-ROS-Filesystem.md.md)


