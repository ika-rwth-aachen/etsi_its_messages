cmake_minimum_required(VERSION 3.5)
project(etsi_its_msgs)

find_package(ament_cmake REQUIRED)
find_package(etsi_its_cam_msgs REQUIRED)
find_package(etsi_its_cam_ts_msgs REQUIRED)
find_package(etsi_its_cpm_ts_msgs REQUIRED)
find_package(etsi_its_denm_msgs REQUIRED)
find_package(etsi_its_denm_ts_msgs REQUIRED)
find_package(etsi_its_ivim_ts_msgs REQUIRED)
find_package(etsi_its_mapem_ts_msgs REQUIRED)
find_package(etsi_its_spatem_ts_msgs REQUIRED)
find_package(etsi_its_vam_ts_msgs REQUIRED)
find_package(etsi_its_rtcmem_ts_msgs REQUIRED)

# aggregate target so that downstream packages can link against all ETSI ITS message types
add_library(${PROJECT_NAME} INTERFACE)
target_link_libraries(${PROJECT_NAME} INTERFACE
  ${etsi_its_cam_msgs_TARGETS}
  ${etsi_its_cam_ts_msgs_TARGETS}
  ${etsi_its_cpm_ts_msgs_TARGETS}
  ${etsi_its_denm_msgs_TARGETS}
  ${etsi_its_denm_ts_msgs_TARGETS}
  ${etsi_its_ivim_ts_msgs_TARGETS}
  ${etsi_its_mapem_ts_msgs_TARGETS}
  ${etsi_its_spatem_ts_msgs_TARGETS}
  ${etsi_its_vam_ts_msgs_TARGETS}
  ${etsi_its_rtcmem_ts_msgs_TARGETS}
)
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
)
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  etsi_its_cam_msgs
  etsi_its_cam_ts_msgs
  etsi_its_cpm_ts_msgs
  etsi_its_denm_msgs
  etsi_its_denm_ts_msgs
  etsi_its_ivim_ts_msgs
  etsi_its_mapem_ts_msgs
  etsi_its_spatem_ts_msgs
  etsi_its_vam_ts_msgs
  etsi_its_rtcmem_ts_msgs
)

ament_package()
