#pragma once

#include <etsi_its_cam_coding/BasicVehicleContainerLowFrequency.h>
#include <etsi_its_cam_msgs/BasicVehicleContainerLowFrequency.h>
#include <etsi_its_cam_conversion/convertVehicleRole.h>
#include <etsi_its_cam_conversion/convertExteriorLights.h>
#include <etsi_its_cam_conversion/convertPathHistory.h>

namespace etsi_its_cam_conversion {
  
void toRos_BasicVehicleContainerLowFrequency(const BasicVehicleContainerLowFrequency_t& in, etsi_its_cam_msgs::BasicVehicleContainerLowFrequency& out) {
  toRos_VehicleRole(in.vehicleRole, out.vehicleRole);
  toRos_ExteriorLights(in.exteriorLights, out.exteriorLights);
  toRos_PathHistory(in.pathHistory, out.pathHistory);
}

void toStruct_BasicVehicleContainerLowFrequency(const etsi_its_cam_msgs::BasicVehicleContainerLowFrequency& in, BasicVehicleContainerLowFrequency_t& out) {
  memset(&out, 0, sizeof(BasicVehicleContainerLowFrequency_t));
  toStruct_VehicleRole(in.vehicleRole, out.vehicleRole);
  toStruct_ExteriorLights(in.exteriorLights, out.exteriorLights);
  toStruct_PathHistory(in.pathHistory, out.pathHistory);
}

}