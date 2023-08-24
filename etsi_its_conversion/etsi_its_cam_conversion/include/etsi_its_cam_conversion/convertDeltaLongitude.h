#pragma once

#include <etsi_its_cam_coding/DeltaLongitude.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>
#ifdef ROS2
#include <etsi_its_cam_msgs/msg/delta_longitude.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#else
#include <etsi_its_cam_msgs/DeltaLongitude.h>
namespace cam_msgs = etsi_its_cam_msgs;
#endif


namespace etsi_its_cam_conversion {

void toRos_DeltaLongitude(const DeltaLongitude_t& in, cam_msgs::DeltaLongitude& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_DeltaLongitude(const cam_msgs::DeltaLongitude& in, DeltaLongitude_t& out) {

  memset(&out, 0, sizeof(DeltaLongitude_t));
  toStruct_INTEGER(in.value, out);
}

}