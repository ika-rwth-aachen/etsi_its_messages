#pragma once

#include <etsi_its_denm_coding/Latitude.h>
#include <etsi_its_denm_conversion/primitives/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/Latitude.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/latitude.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_Latitude(const Latitude_t& in, denm_msgs::Latitude& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_Latitude(const denm_msgs::Latitude& in, Latitude_t& out) {

  memset(&out, 0, sizeof(Latitude_t));
  toStruct_INTEGER(in.value, out);
}

}