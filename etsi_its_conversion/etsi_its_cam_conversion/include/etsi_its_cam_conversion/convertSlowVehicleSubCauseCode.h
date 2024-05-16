//// INTEGER SlowVehicleSubCauseCode


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/SlowVehicleSubCauseCode.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/SlowVehicleSubCauseCode.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/slow_vehicle_sub_cause_code.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_SlowVehicleSubCauseCode(const SlowVehicleSubCauseCode_t& in, cam_msgs::SlowVehicleSubCauseCode& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_SlowVehicleSubCauseCode(const cam_msgs::SlowVehicleSubCauseCode& in, SlowVehicleSubCauseCode_t& out) {
  memset(&out, 0, sizeof(SlowVehicleSubCauseCode_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
