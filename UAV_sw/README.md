# Essential
### MAVSDK
MAVSDK is the main library used for developing the drone. This library must be compiled.

### uav_ros2_ws
uav_ros2_ws is the the ROS2 code for developing the drone. To use it you must first install ROS2, see guide.


# Non essential

### glfw / imgui
uav_ros2_ws can be run standalone on the Raspberry PI, but it is easier to develop using using a graphical user interface.

GLFW and imgui are libraries used for a uav_gui node in uav_ros2_ws that enables graphics. This is recommended to install if using a virtual machine.

### PX4-Autopilot
PX4-Autopilot is the software running on the UAV. It is basically the entire operating system. We use it here to access Gazebo which is a simulation software. This is recommended to use before deploying code to the uav in real life (dangerous).

### uav
uav is deprecated. This contains old code using udp communication. This is not used anymore after moving to ROS2.

### px4_ros_com
px4_ros_com contains useful messages and nodes for developing towards the px4 software using ROS2. We don't use it but it has been used earlier in development.
