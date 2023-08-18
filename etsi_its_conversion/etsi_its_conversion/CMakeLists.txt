cmake_minimum_required(VERSION 3.5)
project(etsi_its_conversion)

find_package(ros_environment REQUIRED QUIET)
set(ROS_VERSION $ENV{ROS_VERSION})
set(ROS_DISTRO $ENV{ROS_DISTRO})
add_compile_definitions(ROS_VERSION=$ENV{ROS_VERSION})
add_compile_definitions(ROS_DISTRO=$ENV{ROS_DISTRO})


# === ROS1 (CATKIN) ============================================================
if(${ROS_VERSION} EQUAL 1)
  add_compile_options(-std=c++11)

  find_package(catkin REQUIRED COMPONENTS
    etsi_its_cam_conversion
    nodelet
    roscpp
    roslaunch  
    std_msgs
    udp_msgs
  )

  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
      etsi_its_cam_conversion
      nodelet
      roscpp
      std_msgs
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
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
  )

  install(DIRECTORY
    launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

# === ROS2 (AMENT) =============================================================
elseif(${ROS_VERSION} EQUAL 2)

  find_package(ament_cmake REQUIRED)
  find_package(etsi_its_cam_coding REQUIRED)
  find_package(etsi_its_cam_conversion REQUIRED)
  find_package(etsi_its_cam_msgs REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(udp_msgs REQUIRED)

  # add_executable(${PROJECT_NAME}_node src/ConversionNode.cpp)
  # add_executable(${PROJECT_NAME}_generator src/CamGenerator.cpp)

  # set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME ConversionNode)
  # set_target_properties(${PROJECT_NAME}_generator PROPERTIES OUTPUT_NAME CamGenerator)

  # add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  # add_dependencies(${PROJECT_NAME}_generator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

  # ament_target_dependencies(${PROJECT_NAME}_node
  #   ament_cmake
  #   etsi_its_cam_coding
  #   etsi_its_cam_conversion
  #   etsi_its_cam_msgs
  #   rclcpp
  #   std_msgs
  #   udp_msgs
  # )

  # ament_target_dependencies(${PROJECT_NAME}_generator
  #   ament_cmake
  #   etsi_its_cam_coding
  #   etsi_its_cam_conversion
  #   etsi_its_cam_msgs
  #   rclcpp
  #   std_msgs
  #   udp_msgs
  # )

  # target_include_directories(${PROJECT_NAME}_node PUBLIC
  #   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  #   $<INSTALL_INTERFACE:include>)
  # target_compile_features(${PROJECT_NAME}_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

  # target_include_directories(${PROJECT_NAME}_generator PUBLIC
  # $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  # $<INSTALL_INTERFACE:include>)
  # target_compile_features(${PROJECT_NAME}_generator PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


  # install(DIRECTORY
  #   launch
  #   DESTINATION share/${PROJECT_NAME}
  # )

  ament_package()

endif()