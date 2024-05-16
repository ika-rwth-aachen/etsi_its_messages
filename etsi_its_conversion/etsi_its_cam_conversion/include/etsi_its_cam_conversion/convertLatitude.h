//// INTEGER Latitude


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/Latitude.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/Latitude.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/latitude.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_Latitude(const Latitude_t& in, cam_msgs::Latitude& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_Latitude(const cam_msgs::Latitude& in, Latitude_t& out) {
  memset(&out, 0, sizeof(Latitude_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
