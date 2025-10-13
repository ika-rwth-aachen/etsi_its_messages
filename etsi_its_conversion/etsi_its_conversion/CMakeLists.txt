cmake_minimum_required(VERSION 3.5)
project(etsi_its_conversion)

find_package(ament_cmake REQUIRED)
find_package(etsi_its_cam_conversion REQUIRED)
find_package(etsi_its_cam_ts_conversion REQUIRED)
find_package(etsi_its_conversion_services REQUIRED)
find_package(etsi_its_cpm_ts_conversion REQUIRED)
find_package(etsi_its_denm_conversion REQUIRED)
find_package(etsi_its_denm_ts_conversion REQUIRED)
find_package(etsi_its_mapem_ts_conversion REQUIRED)
find_package(etsi_its_mcm_uulm_conversion REQUIRED)
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
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "etsi_its_conversion::Converter"
  EXECUTABLE ${PROJECT_NAME}_node_mt
  EXECUTOR MultiThreadedExecutor
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(${PROJECT_NAME}
  etsi_its_cam_conversion
  etsi_its_cam_ts_conversion
  etsi_its_conversion_services
  etsi_its_cpm_ts_conversion
  etsi_its_denm_conversion
  etsi_its_denm_ts_conversion
  etsi_its_mapem_ts_conversion
  etsi_its_mcm_uulm_conversion
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
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
