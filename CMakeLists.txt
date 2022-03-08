cmake_minimum_required(VERSION 3.0.2)
project(pybullet_ros)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    cv_bridge
    roscpp
    control_toolbox
    controller_manager
    hardware_interface
    transmission_interface
    joint_limits_interface
    urdf
    angles
)

find_package(Eigen3 REQUIRED)
find_package(pybind11 CONFIG)

catkin_package(
    CATKIN_DEPENDS
    roscpp
    std_msgs
    control_toolbox
    controller_manager
    control_toolbox
    hardware_interface
    transmission_interface
    joint_limits_interface
    urdf
    angles
    INCLUDE_DIRS ros_control_interface/include
    LIBRARIES default_robot_hw_sim
    DEPENDS EIGEN3
)

catkin_python_setup()

link_directories(
    ${catkin_LIBRARY_DIRS}
)

include_directories(ros_control_interface/include
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    )

add_library(default_robot_hw_sim ros_control_interface/src/default_robot_hw_sim.cpp)
target_link_libraries(default_robot_hw_sim ${catkin_LIBRARIES})

set(SOURCES
    ros_control_interface/src/pybullet_ros_control_pybind.cpp
    ros_control_interface/src/pybullet_ros_control_plugin.cpp
    )

pybind11_add_module(pybullet_ros_control_plugin_py
    ${SOURCES}
    )
target_link_libraries(pybullet_ros_control_plugin_py PRIVATE ${catkin_LIBRARIES})

add_custom_command(TARGET pybullet_ros_control_plugin_py POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:pybullet_ros_control_plugin_py>
    ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/
    )
