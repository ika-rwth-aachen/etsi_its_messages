cmake_minimum_required(VERSION 2.8.3)
project(etsi_its_messages)

find_package(ros_environment REQUIRED QUIET)
set(ROS_VERSION $ENV{ROS_VERSION})
set(ROS_DISTRO $ENV{ROS_DISTRO})
add_compile_definitions(ROS_VERSION=$ENV{ROS_VERSION})
add_compile_definitions(ROS_DISTRO=$ENV{ROS_DISTRO})

# === ROS1 (CATKIN) ============================================================
if(${ROS_VERSION} EQUAL 1)

    find_package(catkin REQUIRED)
    catkin_metapackage()

# === ROS2 (AMENT) =============================================================
elseif(${ROS_VERSION} EQUAL 2)

    find_package(ament_cmake REQUIRED)

endif()