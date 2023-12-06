cmake_minimum_required(VERSION 3.10.2)
project(uav_cpp)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
#if(NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
#endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(MAVSDK REQUIRED)
find_package(Threads REQUIRED)
find_package(uav_msgs REQUIRED)

# Needed for some reason
ament_export_dependencies(MAVSDK)
ament_export_dependencies(Threads)

# Create uav_main executable
add_executable(uav_main
  src/uav_main/main.cpp
  src/uav_main/kinematics.cpp
  src/uav_main/mav_util.cpp
  src/uav_main/PID.cpp
)

# Link ros2 stuff using ament linker
ament_target_dependencies(uav_main rclcpp uav_msgs)

# Link all the other stuff using regular linker
target_link_libraries(uav_main
    MAVSDK::mavsdk
    Threads::Threads
)

install(TARGETS uav_main DESTINATION lib/${PROJECT_NAME})


# Compile with or without GUI
# On raspberry this should be OFF, otherwise the choice is yours
set(USE_UAV_GUI OFF)

if (USE_UAV_GUI)
  message("compiling GUI")

  find_package(glfw3 REQUIRED)

  # include imgui
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../imgui/src)

  # Create uav_main executable
  add_executable(uav_gui
    src/uav_gui/main.cpp
    src/uav_gui/gui.cpp
    src/uav_gui/win_commands.cpp
    src/uav_gui/win_controller.cpp
    src/uav_gui/win_kinematics.cpp

    # imgui include src relatively lmao
    ../../../imgui/src/imgui/imgui_demo.cpp
    ../../../imgui/src/imgui/imgui_draw.cpp
    ../../../imgui/src/imgui/imgui_impl_glfw.cpp
    ../../../imgui/src/imgui/imgui_impl_opengl3.cpp
    ../../../imgui/src/imgui/imgui_tables.cpp
    ../../../imgui/src/imgui/imgui_widgets.cpp
    ../../../imgui/src/imgui/imgui.cpp
  )

  # Link ros2 stuff using ament linker
  ament_target_dependencies(uav_gui rclcpp uav_msgs)

  # Link all the other stuff using regular linker
  target_link_libraries(uav_gui
    glfw3 dl m GL GLU X11 # graphics stuff
  )
  install(TARGETS uav_gui DESTINATION lib/${PROJECT_NAME})
endif()


# This doesn't matter
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
