cmake_minimum_required(VERSION 3.5)
project(etsi_its_msgs)

find_package(ros_environment REQUIRED QUIET)
set(ROS_VERSION $ENV{ROS_VERSION})

# === ROS 2 (AMENT) ============================================================
if(${ROS_VERSION} EQUAL 2)

  find_package(ament_cmake REQUIRED)
  find_package(etsi_its_cam_msgs REQUIRED)
  find_package(etsi_its_denm_msgs REQUIRED)
  
  ament_export_dependencies(etsi_its_cam_msgs etsi_its_denm_msgs)
  ament_package()

# === ROS (CATKIN) =============================================================
elseif(${ROS_VERSION} EQUAL 1)

  find_package(catkin REQUIRED COMPONENTS
    etsi_its_cam_msgs
    etsi_its_denm_msgs
  )

  catkin_package(
    CATKIN_DEPENDS
      etsi_its_cam_msgs
      etsi_its_denm_msgs
  )

endif()