cmake_minimum_required(VERSION 3.5)
project(etsi_its_cam_conversion)

find_package(ros_environment REQUIRED QUIET)
set(ROS_VERSION $ENV{ROS_VERSION})
set(ROS_DISTRO $ENV{ROS_DISTRO})
add_compile_definitions(ROS_VERSION=$ENV{ROS_VERSION})
add_compile_definitions(ROS_DISTRO=$ENV{ROS_DISTRO})

# === ROS1 (CATKIN) ============================================================
if(${ROS_VERSION} EQUAL 1)

  find_package(catkin REQUIRED COMPONENTS
    etsi_its_cam_coding  
    etsi_its_cam_msgs
  )
  find_package(Boost REQUIRED)

  catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS etsi_its_cam_coding etsi_its_cam_msgs
    DEPENDS Boost
  )

  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
  )

  ## Mark cpp header files for installation
  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

  ## Mark other files for installation (e.g. launch and bag files, etc.)
  install(FILES
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

# === ROS2 (AMENT) =============================================================
elseif(${ROS_VERSION} EQUAL 2)

  add_compile_definitions(ROS2)

  find_package(ament_cmake REQUIRED)
  find_package(Boost REQUIRED)
  find_package(etsi_its_cam_coding REQUIRED)
  find_package(etsi_its_cam_msgs REQUIRED)

  add_library(${PROJECT_NAME} INTERFACE)

  target_include_directories(${PROJECT_NAME} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    /docker-ros/ws/install/etsi_its_cam_coding/include
    /docker-ros/ws/install/etsi_its_cam_msgs/include
  )
  # TODO: fix target_include_directories of dependencies

  ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)

  ament_export_dependencies(
    Boost
    etsi_its_cam_coding 
    etsi_its_cam_msgs
  )

  install(
    DIRECTORY include/
    DESTINATION include
  )

  install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}Targets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib
        INCLUDES DESTINATION include
  )

  ament_package()

endif()