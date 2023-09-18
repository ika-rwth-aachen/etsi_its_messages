#pragma once

#include <etsi_its_cam_coding/BasicVehicleContainerLowFrequency.h>
#include <etsi_its_cam_conversion/convertVehicleRole.h>
#include <etsi_its_cam_conversion/convertExteriorLights.h>
#include <etsi_its_cam_conversion/convertPathHistory.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/BasicVehicleContainerLowFrequency.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/basic_vehicle_container_low_frequency.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_BasicVehicleContainerLowFrequency(const BasicVehicleContainerLowFrequency_t& in, cam_msgs::BasicVehicleContainerLowFrequency& out) {

  toRos_VehicleRole(in.vehicleRole, out.vehicle_role);
  toRos_ExteriorLights(in.exteriorLights, out.exterior_lights);
  toRos_PathHistory(in.pathHistory, out.path_history);
}

void toStruct_BasicVehicleContainerLowFrequency(const cam_msgs::BasicVehicleContainerLowFrequency& in, BasicVehicleContainerLowFrequency_t& out) {

  memset(&out, 0, sizeof(BasicVehicleContainerLowFrequency_t));

  toStruct_VehicleRole(in.vehicle_role, out.vehicleRole);
  toStruct_ExteriorLights(in.exterior_lights, out.exteriorLights);
  toStruct_PathHistory(in.path_history, out.pathHistory);
}

}