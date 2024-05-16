//// INTEGER InformationQuality


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/InformationQuality.h>
#include <etsi_its_denm_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/InformationQuality.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/information_quality.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_InformationQuality(const InformationQuality_t& in, denm_msgs::InformationQuality& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_InformationQuality(const denm_msgs::InformationQuality& in, InformationQuality_t& out) {
  memset(&out, 0, sizeof(InformationQuality_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
