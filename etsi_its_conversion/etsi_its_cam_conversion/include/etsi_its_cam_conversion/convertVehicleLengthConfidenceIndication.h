#pragma once

#include <etsi_its_cam_coding/VehicleLengthConfidenceIndication.h>
#include <etsi_its_cam_msgs/VehicleLengthConfidenceIndication.h>

namespace etsi_its_cam_conversion {
  
void toRos_VehicleLengthConfidenceIndication(const VehicleLengthConfidenceIndication_t& in, etsi_its_cam_msgs::VehicleLengthConfidenceIndication& out) {
  out.value = in;
}

void toStruct_VehicleLengthConfidenceIndication(const etsi_its_cam_msgs::VehicleLengthConfidenceIndication& in, VehicleLengthConfidenceIndication_t& out) {
  memset(&out, 0, sizeof(VehicleLengthConfidenceIndication_t));
  out = in.value;
}

}