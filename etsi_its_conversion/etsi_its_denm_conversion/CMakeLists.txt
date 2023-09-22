cmake_minimum_required(VERSION 3.5)
project(etsi_its_denm_conversion)

find_package(ros_environment REQUIRED QUIET)
set(ROS_VERSION $ENV{ROS_VERSION})

# === ROS2 (AMENT) =============================================================
if(${ROS_VERSION} EQUAL 2)

  add_compile_definitions(ROS2)

  find_package(ament_cmake REQUIRED)
  find_package(Boost REQUIRED)
  find_package(etsi_its_denm_coding REQUIRED)
  find_package(etsi_its_denm_msgs REQUIRED)

  add_library(${PROJECT_NAME} INTERFACE)

  target_include_directories(${PROJECT_NAME} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  )

  target_link_libraries(${PROJECT_NAME} INTERFACE
    ${etsi_its_denm_coding_TARGETS}
    ${etsi_its_denm_msgs_TARGETS}
  )

  ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
  ament_export_dependencies(
    Boost
    etsi_its_denm_coding
    etsi_its_denm_msgs
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

# === ROS1 (CATKIN) ============================================================
elseif(${ROS_VERSION} EQUAL 1)

  add_compile_definitions(ROS1)

  find_package(catkin REQUIRED COMPONENTS
    etsi_its_denm_coding
    etsi_its_denm_msgs
  )
  find_package(Boost REQUIRED)

  catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS etsi_its_denm_coding etsi_its_denm_msgs
    DEPENDS Boost
  )

  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
  )

  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

  install(FILES
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

endif()