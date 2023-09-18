#pragma once

#include <etsi_its_cam_coding/VehicleWidth.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/VehicleWidth.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/vehicle_width.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_VehicleWidth(const VehicleWidth_t& in, cam_msgs::VehicleWidth& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_VehicleWidth(const cam_msgs::VehicleWidth& in, VehicleWidth_t& out) {

  memset(&out, 0, sizeof(VehicleWidth_t));
  toStruct_INTEGER(in.value, out);
}

}