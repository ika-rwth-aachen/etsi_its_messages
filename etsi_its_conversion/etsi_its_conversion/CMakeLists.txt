cmake_minimum_required(VERSION 3.5)
project(etsi_its_conversion)

find_package(ros_environment REQUIRED QUIET)
set(ROS_VERSION $ENV{ROS_VERSION})

# === ROS 2 (AMENT) ============================================================
if(${ROS_VERSION} EQUAL 2)

  add_compile_definitions(ROS2)

  find_package(ament_cmake REQUIRED)
  find_package(etsi_its_cam_conversion REQUIRED)
  find_package(etsi_its_cam_ts_conversion REQUIRED)
  find_package(etsi_its_cpm_ts_conversion REQUIRED)
  find_package(etsi_its_denm_conversion REQUIRED)
  find_package(etsi_its_denm_ts_conversion REQUIRED)
  find_package(etsi_its_mapem_ts_conversion REQUIRED)
  find_package(etsi_its_spatem_ts_conversion REQUIRED)
  find_package(etsi_its_vam_ts_conversion REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_components REQUIRED)
  find_package(udp_msgs REQUIRED)

  add_library(${PROJECT_NAME} SHARED src/Converter.cpp)

  rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "etsi_its_conversion::Converter"
    EXECUTABLE ${PROJECT_NAME}_node
  )

  target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)

  ament_target_dependencies(${PROJECT_NAME}
    etsi_its_cam_conversion
    etsi_its_cam_ts_conversion
    etsi_its_cpm_ts_conversion
    etsi_its_denm_conversion
    etsi_its_denm_ts_conversion
    etsi_its_mapem_ts_conversion
    etsi_its_spatem_ts_conversion
    etsi_its_vam_ts_conversion
    rclcpp
    rclcpp_components
    udp_msgs
  )

  install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "*ros1*" EXCLUDE
  )

  install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
    PATTERN "*ros1*" EXCLUDE
  )

  ament_package()

# === ROS (CATKIN) =============================================================
elseif(${ROS_VERSION} EQUAL 1)

  add_compile_definitions(ROS1)

  add_compile_options(-std=c++11)

  find_package(catkin REQUIRED COMPONENTS
    etsi_its_cam_conversion
    etsi_its_cam_ts_conversion
    etsi_its_cpm_ts_conversion
    etsi_its_denm_conversion
    etsi_its_denm_ts_conversion
    etsi_its_mapem_ts_conversion
    etsi_its_spatem_ts_conversion
    etsi_its_vam_ts_conversion
    nodelet
    roscpp
    udp_msgs
  )

  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
      etsi_its_cam_conversion
      etsi_its_cam_ts_conversion
      etsi_its_cpm_ts_conversion
      etsi_its_denm_conversion
      etsi_its_denm_ts_conversion
      etsi_its_mapem_ts_conversion
      etsi_its_spatem_ts_conversion
      etsi_its_vam_ts_conversion
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

  target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
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

  install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    FILES_MATCHING PATTERN "*ros1*"
  )

  install(DIRECTORY config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    FILES_MATCHING PATTERN "*ros1*"
  )

  install(FILES nodelet_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

endif()
