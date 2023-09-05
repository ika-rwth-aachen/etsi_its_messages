cmake_minimum_required(VERSION 3.5)
project(etsi_its_coding)

find_package(ros_environment REQUIRED QUIET)
set(ROS_VERSION $ENV{ROS_VERSION})

# === ROS1 (CATKIN) ============================================================
if(${ROS_VERSION} EQUAL 1)

    find_package(catkin REQUIRED)
    catkin_metapackage()

# === ROS2 (AMENT) =============================================================
elseif(${ROS_VERSION} EQUAL 2)

    find_package(ament_cmake REQUIRED)
    ament_package()

endif()
