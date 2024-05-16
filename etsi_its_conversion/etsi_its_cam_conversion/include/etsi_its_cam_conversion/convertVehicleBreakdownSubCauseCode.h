//// INTEGER VehicleBreakdownSubCauseCode


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/VehicleBreakdownSubCauseCode.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/VehicleBreakdownSubCauseCode.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/vehicle_breakdown_sub_cause_code.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_VehicleBreakdownSubCauseCode(const VehicleBreakdownSubCauseCode_t& in, cam_msgs::VehicleBreakdownSubCauseCode& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_VehicleBreakdownSubCauseCode(const cam_msgs::VehicleBreakdownSubCauseCode& in, VehicleBreakdownSubCauseCode_t& out) {
  memset(&out, 0, sizeof(VehicleBreakdownSubCauseCode_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
