cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(uav_gui)

# include shared src
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../shared)

# include glfw
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../glfw/glfw/include)

# include imgui
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../imgui/src)

# find glfw lib
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../glfw/glfw/build/src)

add_executable(uav_gui
    src/main.cpp
    src/gui.cpp
    src/win_controller.cpp
    src/win_kinematics.cpp
    src/win_commands.cpp

    # shared src files
    ../shared/uav/udp.cpp

    # imgui files
    ../../imgui/src/imgui/imgui_demo.cpp
    ../../imgui/src/imgui/imgui_draw.cpp
    ../../imgui/src/imgui/imgui_impl_glfw.cpp
    ../../imgui/src/imgui/imgui_impl_opengl3.cpp
    ../../imgui/src/imgui/imgui_tables.cpp
    ../../imgui/src/imgui/imgui_widgets.cpp
    ../../imgui/src/imgui/imgui.cpp
)

find_package(MAVSDK REQUIRED)
#find_package(SDL2 REQUIRED)
find_package(Threads REQUIRED)

# Workaround for Ubuntu 18.04 where SDL2::SDL2 isn't provided yet.
#if (SDL2_FOUND AND NOT TARGET SDL2::SDL2)
#    add_library(SDL2::SDL2 UNKNOWN IMPORTED)
#    set_target_properties(SDL2::SDL2 PROPERTIES
#        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
#        IMPORTED_LOCATION "${SDL2_LIBRARIES}"
#        INTERFACE_INCLUDE_DIRECTORIES "${SDL2_INCLUDE_DIRS}"
#    )
#endif()


# link against libraries
target_link_libraries(uav_gui
    MAVSDK::mavsdk
#    SDL2::SDL2
    Threads::Threads
    
    # All of these below are related to glfw and the order matters
    glfw3
    dl
    m
    GL
    GLU
    X11
)

if(NOT MSVC)
    add_compile_options(uav_gui PRIVATE -Wall -Wextra)
else()
    add_compile_options(uav_gui PRIVATE -WX -W2)
endif()
