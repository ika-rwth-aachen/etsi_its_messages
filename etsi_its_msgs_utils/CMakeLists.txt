cmake_minimum_required(VERSION 3.5)
project(etsi_its_msgs_utils)

include(${PROJECT_NAME}-extras.cmake)

find_package(ament_cmake REQUIRED)
find_package(etsi_its_msgs REQUIRED)
find_package(GeographicLib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(${PROJECT_NAME} INTERFACE
  ${etsi_its_msgs_TARGETS}
  ${GeographicLib_LIBRARIES}
  ${geometry_msgs_TARGETS}
  ${tf2_geometry_msgs_TARGETS}
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(etsi_its_msgs geometry_msgs tf2_geometry_msgs)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# install all sample python scripts
file(GLOB SAMPLE_SCRIPTS "samples/publish_*.py")
install(
  FILES "samples/utils.py"
  DESTINATION lib/${PROJECT_NAME}
)
install(
  PROGRAMS ${SAMPLE_SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}
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
  ament_add_gtest(${PROJECT_NAME}-test test/test_access.cpp)
  target_include_directories(${PROJECT_NAME}-test PUBLIC test
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${PROJECT_NAME}-test
    etsi_its_msgs
    GeographicLib
    geometry_msgs
    tf2_geometry_msgs
  )
endif()

ament_package(
  CONFIG_EXTRAS ${PROJECT_NAME}-extras.cmake
)
