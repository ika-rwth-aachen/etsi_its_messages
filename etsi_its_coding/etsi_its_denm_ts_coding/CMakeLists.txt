cmake_minimum_required(VERSION 3.5)
project(etsi_its_denm_ts_coding)

AUX_SOURCE_DIRECTORY(src SRC_FILES)

find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME} SHARED
  ${SRC_FILES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)

install(DIRECTORY include/
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
