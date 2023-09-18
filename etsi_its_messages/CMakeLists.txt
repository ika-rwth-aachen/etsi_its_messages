cmake_minimum_required(VERSION 3.5)
project(etsi_its_messages)

find_package(ros_environment REQUIRED QUIET)
set(ROS_VERSION $ENV{ROS_VERSION})

# === ROS2 (AMENT) =============================================================
if(${ROS_VERSION} EQUAL 2)

  find_package(ament_cmake REQUIRED)
  ament_package()

# === ROS1 (CATKIN) ============================================================
elseif(${ROS_VERSION} EQUAL 1)

  find_package(catkin REQUIRED)
  catkin_metapackage()

endif()
