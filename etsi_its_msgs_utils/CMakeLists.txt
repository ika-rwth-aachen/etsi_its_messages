cmake_minimum_required(VERSION 3.5)
project(etsi_its_msgs_utils)

find_package(ros_environment REQUIRED QUIET)
set(ROS_VERSION $ENV{ROS_VERSION})

include(${PROJECT_NAME}-extras.cmake)

# === ROS 2 (AMENT) ============================================================
if(${ROS_VERSION} EQUAL 2)

  find_package(ament_cmake REQUIRED)
  find_package(etsi_its_msgs REQUIRED)
  find_package(GeographicLib REQUIRED)
  find_package(geometry_msgs REQUIRED)

  add_library(${PROJECT_NAME} INTERFACE)

  target_include_directories(${PROJECT_NAME} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  )

  target_link_libraries(${PROJECT_NAME} INTERFACE
    ${etsi_its_msgs_TARGETS}
    ${GeographicLib_LIBRARIES}
    ${geometry_msgs_TARGETS}
  )

  ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
  ament_export_dependencies(etsi_its_msgs geometry_msgs)

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

  if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(${PROJECT_NAME}-cam_test test/test_cam_access.ros2.cpp)
    target_include_directories(${PROJECT_NAME}-cam_test PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    target_include_directories(${PROJECT_NAME}-cam_test PUBLIC test)
    ament_target_dependencies(${PROJECT_NAME}-cam_test
      etsi_its_msgs
      GeographicLib
      geometry_msgs
    )
    ament_add_gtest(${PROJECT_NAME}-denm_test test/test_denm_access.ros2.cpp)
    target_include_directories(${PROJECT_NAME}-denm_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    )
    target_include_directories(${PROJECT_NAME}-denm_test PUBLIC test)
    ament_target_dependencies(${PROJECT_NAME}-denm_test
      etsi_its_msgs
      GeographicLib
      geometry_msgs
    )
  endif()

  ament_package(
    CONFIG_EXTRAS ${PROJECT_NAME}-extras.cmake
  )

# === ROS (CATKIN) =============================================================
elseif(${ROS_VERSION} EQUAL 1)

  find_package(catkin REQUIRED COMPONENTS
    etsi_its_msgs
    geometry_msgs
  )

  find_package(GeographicLib REQUIRED)

  catkin_package(
    CFG_EXTRAS ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}-extras.cmake
    INCLUDE_DIRS include
    CATKIN_DEPENDS
      etsi_its_msgs
      geometry_msgs
  )

  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${GeographicLib_INCLUDE_DIRS}
  )

  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

  if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(${PROJECT_NAME}-cam_test test/test_cam_access.cpp)
    catkin_add_gtest(${PROJECT_NAME}-denm_test test/test_denm_access.cpp)
    if(TARGET ${PROJECT_NAME}-cam_test)
      target_include_directories(${PROJECT_NAME}-cam_test PUBLIC test)
      target_link_libraries(${PROJECT_NAME}-cam_test ${GeographicLib_LIBRARIES} ${catkin_LIBRARIES})
    endif()
    if(TARGET ${PROJECT_NAME}-denm_test)
      target_include_directories(${PROJECT_NAME}-denm_test PUBLIC test)
      target_link_libraries(${PROJECT_NAME}-denm_test ${GeographicLib_LIBRARIES} ${catkin_LIBRARIES})
    endif()
  endif()

endif()
