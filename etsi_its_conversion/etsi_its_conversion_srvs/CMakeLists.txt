cmake_minimum_required(VERSION 3.5)
project(etsi_its_conversion_srvs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(etsi_its_cam_msgs REQUIRED)
find_package(etsi_its_denm_msgs REQUIRED)
find_package(udp_msgs REQUIRED)

# Services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ConvertCamToUdp.srv"
  "srv/ConvertUdpToCam.srv"
  "srv/ConvertDenmToUdp.srv"
  "srv/ConvertUdpToDenm.srv"
  DEPENDENCIES etsi_its_cam_msgs etsi_its_denm_msgs udp_msgs
)

ament_export_dependencies(rosidl_default_runtime)

#target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)


ament_package()
