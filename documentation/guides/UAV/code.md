# Code guide

This guide covers the code written for the UAV.

The communication folder in this repository is currently not used in the UAV.

The idea was to compare the IMU differences of the UGV and UAV and interpolate between the detection frames from the vision machine learning model.

Apart from that, the code can be found in uav_ros2_ws. The uav directory is currently outdated and not in use.

## Python

The python code is in the uav_ros2_ws/src/uav_py/uav_py directory.

In there you can find the vision_node.py file. This is the node that reads camera, detects the targets and sends the data to the uav_main node (the brain).

You need access to the internet the first time you run vision_node because the machine learning model is downloaded from roboflow. It is probably cached somewhere after that because after this, no connection is required.

This code does no processing on it self. The uav_main node interprets the data accordingly.

vision_node must be started before uav_main because uav_main expects a data stream from this node. If you start uav_main first, vision_topic will not exist when uav_main expects it.

## C++

The c++ code is in the uav_ros2_ws/src/uav_cpp/src directory.

In there you can find 2 folders, uav_main and uav_gui.

### uav_main

uav_main is the main code. It is split up into several files. The main.cpp file runs first but to not make it too big the code is distributed over multiple files. MAVSDK is used in this code so you must compile that first, see guide.

kinematics is a class that interprets IMU acceleration and angular velocity and tries to integrate it over time to predict velocity and position. In practise this is works horribly and should not be used.

mav_util is just MAVSDK code moved away from main to make it easier to read. The main file was difficult to read before this.

PID is a pid controller. It is probably not working correcly and not used. It was supposed to be used for an autonomous algorithm.

vision.h contains the vision algorithm to predict position, angle and depth of the data that is sent from vision_node.

### uav_gui

uav_gui is a Graphical User Interface (GUI) that can be used along uav_main.

This was created to allow easier development, especially when using gazebo simulation.

To use this GLFW must first be compiled and installed, see guide.

All files starting with win_ constains code for some window in the GUI.

The uav_gui node can not be used by itself. Instead it communicates with uav_main.

This can not be used on the raspberry pi since we are using a ubuntu server OS which does not have a windowing system such as x11.

By default, this code is not compiled. To change that, you must change this in the CMakeLists.txt ...

```cmake
set(USE_UAV_GUI OFF)
```

... to

```cmake
set(USE_UAV_GUI ON)
```