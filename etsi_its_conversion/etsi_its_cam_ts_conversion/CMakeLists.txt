cmake_minimum_required(VERSION 3.5)
project(etsi_its_cam_ts_conversion)

find_package(ros_environment REQUIRED QUIET)
set(ROS_VERSION $ENV{ROS_VERSION})

# === ROS 2 (AMENT) ============================================================
if(${ROS_VERSION} EQUAL 2)

  add_compile_definitions(ROS2)

  find_package(ament_cmake REQUIRED)
  find_package(etsi_its_cam_ts_coding REQUIRED)
  find_package(etsi_its_cam_ts_msgs REQUIRED)
  find_package(etsi_its_primitives_conversion REQUIRED)

  add_library(${PROJECT_NAME} INTERFACE)

  target_include_directories(${PROJECT_NAME} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  )

  target_link_libraries(${PROJECT_NAME} INTERFACE
    ${etsi_its_cam_ts_coding_TARGETS}
    ${etsi_its_cam_ts_msgs_TARGETS}
    ${etsi_its_primitives_conversion_TARGETS}
  )

  ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
  ament_export_dependencies(
    etsi_its_cam_ts_coding
    etsi_its_cam_ts_msgs
    etsi_its_primitives_conversion
  )

  install(DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
  )

  install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib
    INCLUDES DESTINATION include
  )

  ament_package()

# === ROS (CATKIN) =============================================================
elseif(${ROS_VERSION} EQUAL 1)

  add_compile_definitions(ROS1)

  find_package(catkin REQUIRED COMPONENTS
    etsi_its_cam_ts_coding
    etsi_its_cam_ts_msgs
    etsi_its_primitives_conversion
  )

  catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS etsi_its_cam_ts_coding etsi_its_cam_ts_msgs etsi_its_primitives_conversion
  )

  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
  )

  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

  install(FILES
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

endif()
