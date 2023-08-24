cmake_minimum_required(VERSION 3.5)
project(etsi_its_conversion)

find_package(ros_environment REQUIRED QUIET)
set(ROS_VERSION $ENV{ROS_VERSION})
set(ROS_DISTRO $ENV{ROS_DISTRO})


# === ROS1 (CATKIN) ============================================================
if(${ROS_VERSION} EQUAL 1)

  add_compile_options(-std=c++11)

  find_package(catkin REQUIRED COMPONENTS
    etsi_its_cam_conversion
    etsi_its_cam_msgs
    nodelet
    roscpp
    udp_msgs
  )

  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
      etsi_its_cam_conversion
      etsi_its_cam_msgs
      nodelet
      roscpp
      udp_msgs
  )

  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
  )

  add_library(${PROJECT_NAME}
    src/Converter.cpp
  )

  add_executable(${PROJECT_NAME}_node src/ConverterNode.cpp)

  set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME converter PREFIX "")

  target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )
  target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
  )

  install(TARGETS ${PROJECT_NAME}_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.hpp"
    PATTERN ".svn" EXCLUDE
  )

  install(DIRECTORY
    launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    FILES_MATCHING PATTERN "*ros1*"
  )

  install(DIRECTORY
    config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    FILES_MATCHING PATTERN "*ros1*"
  )

# === ROS2 (AMENT) =============================================================
elseif(${ROS_VERSION} EQUAL 2)

  add_compile_definitions(ROS2)

  find_package(ament_cmake REQUIRED)
  find_package(etsi_its_cam_coding REQUIRED) # TODO: should not be needed here as it is a dependency of etsi_its_cam_conversion
  find_package(etsi_its_cam_conversion REQUIRED)
  find_package(etsi_its_cam_msgs REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(udp_msgs REQUIRED)

  add_executable(${PROJECT_NAME} src/Converter.cpp)

  ament_target_dependencies(${PROJECT_NAME}
    etsi_its_cam_coding # TODO: should not be needed here as it is a dependency of etsi_its_cam_conversion
    etsi_its_cam_conversion
    etsi_its_cam_msgs
    rclcpp
    udp_msgs
  )

  target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)

  install(
    TARGETS ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
  )

  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "*ros1*" EXCLUDE
  )

  install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
    PATTERN "*ros1*" EXCLUDE
  )

  ament_package()

endif()