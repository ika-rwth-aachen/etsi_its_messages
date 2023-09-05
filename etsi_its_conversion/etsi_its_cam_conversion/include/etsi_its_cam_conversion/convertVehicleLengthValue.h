#pragma once

#include <etsi_its_cam_coding/VehicleLengthValue.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/VehicleLengthValue.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/vehicle_length_value.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_VehicleLengthValue(const VehicleLengthValue_t& in, cam_msgs::VehicleLengthValue& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_VehicleLengthValue(const cam_msgs::VehicleLengthValue& in, VehicleLengthValue_t& out) {

  memset(&out, 0, sizeof(VehicleLengthValue_t));
  toStruct_INTEGER(in.value, out);
}

}