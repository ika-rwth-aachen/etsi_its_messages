#pragma once

#include <etsi_its_cam_coding/BasicVehicleContainerLowFrequency.h>
#include <etsi_its_cam_conversion/convertVehicleRole.h>
#include <etsi_its_cam_conversion/convertExteriorLights.h>
#include <etsi_its_cam_conversion/convertPathHistory.h>
#include <etsi_its_cam_msgs/BasicVehicleContainerLowFrequency.h>


namespace etsi_its_cam_conversion {

void toRos_BasicVehicleContainerLowFrequency(const BasicVehicleContainerLowFrequency_t& in, etsi_its_cam_msgs::BasicVehicleContainerLowFrequency& out) {

  toRos_VehicleRole(in.vehicle_role, out.vehicle_role);
  toRos_ExteriorLights(in.exterior_lights, out.exterior_lights);
  toRos_PathHistory(in.path_history, out.path_history);
}

void toStruct_BasicVehicleContainerLowFrequency(const etsi_its_cam_msgs::BasicVehicleContainerLowFrequency& in, BasicVehicleContainerLowFrequency_t& out) {
    
  memset(&out, 0, sizeof(BasicVehicleContainerLowFrequency_t));

  toStruct_VehicleRole(in.vehicle_role, out.vehicle_role);
  toStruct_ExteriorLights(in.exterior_lights, out.exterior_lights);
  toStruct_PathHistory(in.path_history, out.path_history);
}

}