cmake_minimum_required(VERSION 3.5)
project(etsi_its_ivim_ts_conversion)

find_package(ament_cmake REQUIRED)
find_package(etsi_its_ivim_ts_coding REQUIRED)
find_package(etsi_its_ivim_ts_msgs REQUIRED)
find_package(etsi_its_primitives_conversion REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(${PROJECT_NAME} INTERFACE
  ${etsi_its_ivim_ts_coding_TARGETS}
  ${etsi_its_ivim_ts_msgs_TARGETS}
  ${etsi_its_primitives_conversion_TARGETS}
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  etsi_its_ivim_ts_coding
  etsi_its_ivim_ts_msgs
  etsi_its_primitives_conversion
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/generate_conversion_headers.sh
  scripts/audit_conversion_coverage.py
  scripts/verify_codegen_and_build.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
  INCLUDES DESTINATION include
)

ament_package()


enable_testing()

add_test(NAME ivim_conversion_coverage_test
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_ivim_conversion_coverage.py)
