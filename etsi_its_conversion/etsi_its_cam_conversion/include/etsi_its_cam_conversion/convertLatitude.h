#pragma once

#include <etsi_its_cam_coding/Latitude.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/Latitude.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/latitude.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_Latitude(const Latitude_t& in, cam_msgs::Latitude& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_Latitude(const cam_msgs::Latitude& in, Latitude_t& out) {

  memset(&out, 0, sizeof(Latitude_t));
  toStruct_INTEGER(in.value, out);
}

}