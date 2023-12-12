# ROS2

## Installation

Run the following commands for a full installation

```bash
locale (verify that we use utf-8)
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade
sudo apt install ros-foxy-ros-base python3-argcomplete
sudo apt install ros-dev-tools
```

Every time you want to use ROS2 you have to source it again with this command:

```bash
source /opt/ros/foxy/setup.bash
```

It is advisable to add it to the .bashrc file in ubntu. Then this command will run when ubuntu is booting every time.

## PX4 messaging

To be able to use px4 specific ros messages, px4_msgs is needed. Additionally px4_ros_com is useful as it contains code examples.

```bash
mkdir -p ~/px4_ros2_ws/src/
cd ~/px4_ros2_ws/src/ 

git clone https://github.com/PX4/px4_msgs.git  
git clone https://github.com/PX4/px4_ros_com.git  
 
cd ..

source /opt/ros/foxy/setup.bash 

colcon build

sudo apt install pip 
pip install --user -U empy pyros-genmsg setuptools 
```

Then to use these nodes and messages you need to source the install folder. Run this and add it to .bashrc:

```bash
source ~/px4_ros2_ws/install/setup.bash
```

## Creating ROS2 packages

The following steps shows how uav_ros2_ws was created.

Modify and adapt for your use case if you want custom packages.

Here we create uav_py, uav_cpp and uav_msgs.

Assuming that you are in the home folder, to the following steps.

```bash
mkdir uav_ros2_ws
cd uav_ros2_ws/
mkdir src
cd src/
ros2 pkg create uav_py --build-type ament_python --dependencies rclpy
ros2 pkg create uav_cpp --build-type ament_cmake --dependencies rclcpp
ros2 pkg create --build-type ament_cmake uav_msgs
cd .. 
colcon build
```

add following two lines in ~/.bashrc file 

```bash
source ~/uav_ros2_ws/src/install/setup.bash
source /opt/ros/foxy/setup.bash
```

Them source .bashrc to apply changes now.

```bash
source ~/.bashrc
```

### Python package

Create python files and change mode to execution rights

```bash
cd ~/uav_ros2_ws/src/uav_py/uav_py
touch your_python_file1.py
touch your_python_file2.py
chmod +x your_python_file1.py
chmod +x your_python_file2.py
```

Whenever you create a new python file you must also add it to entry_points in setup.py

In this case, add this:

```bash
entry_points={
    'console_scripts': [
    'your_python_node1 = uav_py.your_python_file1:main',
    'your_python_node2 = uav_py.your_python_file2:main'
    ],
},
```

write your code in these files and then build it using:

```bash
cd ~/uav_ros2_ws
colcon build --packages-select uav_py
```

You always compile your files from the workspace folder as shown above.

Run a node like this.

```bash
ros2 run uav_py your_python_node1
```
### C++ package

Add your cpp file

```bash
cd ~/uav_ros2_ws/src/uav_cpp/src
touch your_cpp_file1.py
```

Then you must add stuff in CMakeLists.txt so that it compiles properly.

First you must find ros2 packages:

```cmake
find_package(rclcpp REQUIRED)
```

Then add an executable where you add all your cpp files. In this case:

```cmake
add_executable(your_cpp_executable
  src/your_cpp_file1.cpp
)
```

Then you link ros2 stuff

```cmake
ament_target_dependencies(your_cpp_executable rclcpp)
```

Then you install it with:

```cmake
install(TARGETS your_cpp_executable DESTINATION lib/${PROJECT_NAME})
```

And then you can build it with:

```bash
cd ~/uav_ros2_ws
colcon build --packages-select uav_cpp
```

Run a node like this.

```bash
ros2 run uav_cpp your_cpp_executable
```

### Message package

```bash
cd ~/uav_ros2_ws/src/uav_msgs
mkdir msg
cd msg
```

For this example we will create the vision message that is with the image processing algorithm:

```bash
touch Vision.msg
```

In this file, the following were added. These are the things we want to send in the message.

```bash
int32 red_x
int32 red_y
int32 red_width
int32 red_height
int32 blue_x
int32 blue_y
int32 blue_width
int32 blue_height
float32 time_seconds
```

To compile you must add the following to CMakeLists.txt...

```bash
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Vision.msg"
 )
```

...and add the following to package.xml

```xml
<build_depend>rosidl_default_generators</build_depend>

<exec_depend>rosidl_default_runtime</exec_depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```

It is easy to get errors here. Make sure everything is sourced like it should be.

You can build it with:

```bash
cd ~/uav_ros2_ws
colcon build --packages-select uav_msgs
```

In python you can access this message with:

```python
from uav_msgs.msg import Vision
```

In python you can access this message with:

```c++
#include <uav_msgs/msg/vision.hpp>
```

In c++ you must link towards this message package before you can include it in code.

You need the following in the uav_cpp CMakeLists.txt

```cmake
find_package(uav_msgs REQUIRED)

ament_target_dependencies(your_cpp_executable uav_msgs)
```

### ( -_-)..zzZZ

As a final note, you can compile all packages in a workspace at the same time with:

```bash
cd ~/uav_ros2_ws
colcon build
```