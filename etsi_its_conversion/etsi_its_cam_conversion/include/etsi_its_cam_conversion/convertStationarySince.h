//// ENUMERATED StationarySince


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/StationarySince.h>

#ifdef ROS1
#include <etsi_its_cam_msgs/StationarySince.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/stationary_since.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_StationarySince(const StationarySince_t& in, cam_msgs::StationarySince& out) {
  out.value = in;
}

void toStruct_StationarySince(const cam_msgs::StationarySince& in, StationarySince_t& out) {
  memset(&out, 0, sizeof(StationarySince_t));

  out = in.value;
}

}
