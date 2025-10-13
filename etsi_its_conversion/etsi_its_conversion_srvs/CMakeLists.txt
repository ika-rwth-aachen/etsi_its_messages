cmake_minimum_required(VERSION 3.5)
project(etsi_its_conversion_srvs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(etsi_its_cam_msgs REQUIRED)
find_package(etsi_its_cam_ts_msgs REQUIRED)
find_package(etsi_its_cpm_ts_msgs REQUIRED)
find_package(etsi_its_denm_msgs REQUIRED)
find_package(etsi_its_denm_ts_msgs REQUIRED)
find_package(etsi_its_mapem_ts_msgs REQUIRED)
find_package(etsi_its_mcm_uulm_msgs REQUIRED)
find_package(etsi_its_spatem_ts_msgs REQUIRED)
find_package(etsi_its_vam_ts_msgs REQUIRED)

find_package(udp_msgs REQUIRED)

# Services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ConvertCamToUdp.srv"
  "srv/ConvertCamTsToUdp.srv"
  "srv/ConvertCpmTsToUdp.srv"
  "srv/ConvertDenmToUdp.srv"
  "srv/ConvertDenmTsToUdp.srv"
  "srv/ConvertMapemTsToUdp.srv"
  "srv/ConvertMcmUulmToUdp.srv"
  "srv/ConvertSpatemTsToUdp.srv"
  "srv/ConvertUdpToCam.srv"
  "srv/ConvertUdpToCamTs.srv"
  "srv/ConvertUdpToCpmTs.srv"
  "srv/ConvertUdpToDenm.srv"
  "srv/ConvertUdpToDenmTs.srv"
  "srv/ConvertUdpToMapemTs.srv"
  "srv/ConvertUdpToMcmUulm.srv"
  "srv/ConvertUdpToSpatemTs.srv"
  "srv/ConvertUdpToVamTs.srv"
  "srv/ConvertVamTsToUdp.srv"
  DEPENDENCIES etsi_its_cam_msgs etsi_its_cam_ts_msgs etsi_its_cpm_ts_msgs etsi_its_denm_msgs etsi_its_denm_ts_msgs etsi_its_mapem_ts_msgs etsi_its_mcm_uulm_msgs etsi_its_spatem_ts_msgs etsi_its_vam_ts_msgs udp_msgs
)

ament_export_dependencies(rosidl_default_runtime)

#target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)


ament_package()
