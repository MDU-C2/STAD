mkdir drone
cd drone/
mkdir src
cd src/
ros2 pkg create pythonpkg --build-type ament_python --dependencies rclpy
ros2 pkg create cpppkg --build-type ament_cmake --dependencies rclcpp
cd .. 
colcon build 

append following two lines in ~/.bashrc file 
source ~/ros/src/install/setup.bash
source /opt/ros/foxy/setup.bash

source ~/.bashrc
cd ~/drone/src/pythonpkg/pythonpkg

touch camera_node.py
touch husky_imu_node.py
touch emergency_breake_node.py
touch flight_controller_node.py

chmod +x camera_node.py
chmod +x husky_imu_node.py
chmod +x emergency_breake_node.py
chmod +x flight_controller_node.py

write your code in these files.

cd ~/drone
colcon build 

#run nodes with the following command
ros2 run pypkg camera_node