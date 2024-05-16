//// INTEGER Latitude


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/Latitude.h>
#include <etsi_its_denm_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/Latitude.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/latitude.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_Latitude(const Latitude_t& in, denm_msgs::Latitude& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_Latitude(const denm_msgs::Latitude& in, Latitude_t& out) {
  memset(&out, 0, sizeof(Latitude_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
