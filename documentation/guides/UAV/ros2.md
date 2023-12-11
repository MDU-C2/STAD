# ROS2

### Installation

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

### PX4 messaging

To be able to use px4 specific ros messages, px4_msgs is needed. Additionally px4_ros_com is useful as it contains code examples.

```bash
mkdir -p ~/px4_ros2_ws/src/
cd ~/px4_ros2_ws/src/ 

git clone https://github.com/PX4/px4_msgs.git  
git clone https://github.com/PX4/px4_ros_com.git  
 
cd ..

source /opt/ros/foxy/setup.bash 

colcon build
```

Then to use these nodes and messages you need to source the install folder. Run this and add it to .bashrc:

```bash
source ~/px4_ros2_ws/install/setup.bash
```
