#pragma once

#include <etsi_its_cam_coding/Longitude.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/Longitude.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/longitude.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_Longitude(const Longitude_t& in, cam_msgs::Longitude& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_Longitude(const cam_msgs::Longitude& in, Longitude_t& out) {

  memset(&out, 0, sizeof(Longitude_t));
  toStruct_INTEGER(in.value, out);
}

}