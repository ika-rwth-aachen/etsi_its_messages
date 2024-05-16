//// INTEGER EmergencyVehicleApproachingSubCauseCode


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/EmergencyVehicleApproachingSubCauseCode.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/EmergencyVehicleApproachingSubCauseCode.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/emergency_vehicle_approaching_sub_cause_code.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_EmergencyVehicleApproachingSubCauseCode(const EmergencyVehicleApproachingSubCauseCode_t& in, cam_msgs::EmergencyVehicleApproachingSubCauseCode& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_EmergencyVehicleApproachingSubCauseCode(const cam_msgs::EmergencyVehicleApproachingSubCauseCode& in, EmergencyVehicleApproachingSubCauseCode_t& out) {
  memset(&out, 0, sizeof(EmergencyVehicleApproachingSubCauseCode_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
