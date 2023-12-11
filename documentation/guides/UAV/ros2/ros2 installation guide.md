# ROS2 installation guide

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
