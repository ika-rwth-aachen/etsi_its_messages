#pragma once

#include <etsi_its_denm_coding/Longitude.h>
#include <etsi_its_denm_conversion/primitives/convertINTEGER.h>
#ifdef ROS2
#include <etsi_its_denm_msgs/msg/longitude.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#else
#include <etsi_its_denm_msgs/Longitude.h>
namespace denm_msgs = etsi_its_denm_msgs;
#endif


namespace etsi_its_denm_conversion {

void toRos_Longitude(const Longitude_t& in, denm_msgs::Longitude& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_Longitude(const denm_msgs::Longitude& in, Longitude_t& out) {

  memset(&out, 0, sizeof(Longitude_t));
  toStruct_INTEGER(in.value, out);
}

}