#pragma once

#include <etsi_its_denm_coding/InformationQuality.h>
#include <etsi_its_denm_conversion/primitives/convertINTEGER.h>
#ifdef ROS2
#include <etsi_its_denm_msgs/msg/information_quality.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#else
#include <etsi_its_denm_msgs/InformationQuality.h>
namespace denm_msgs = etsi_its_denm_msgs;
#endif


namespace etsi_its_denm_conversion {

void toRos_InformationQuality(const InformationQuality_t& in, denm_msgs::InformationQuality& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_InformationQuality(const denm_msgs::InformationQuality& in, InformationQuality_t& out) {

  memset(&out, 0, sizeof(InformationQuality_t));
  toStruct_INTEGER(in.value, out);
}

}