#pragma once

#include <etsi_its_denm_coding/Longitude.h>
#include <etsi_its_denm_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/Longitude.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/longitude.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_Longitude(const Longitude_t& in, denm_msgs::Longitude& out) {

  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_Longitude(const denm_msgs::Longitude& in, Longitude_t& out) {

  memset(&out, 0, sizeof(Longitude_t));
  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}