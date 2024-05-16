//// INTEGER RoadworksSubCauseCode


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/RoadworksSubCauseCode.h>
#include <etsi_its_denm_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/RoadworksSubCauseCode.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/roadworks_sub_cause_code.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_RoadworksSubCauseCode(const RoadworksSubCauseCode_t& in, denm_msgs::RoadworksSubCauseCode& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_RoadworksSubCauseCode(const denm_msgs::RoadworksSubCauseCode& in, RoadworksSubCauseCode_t& out) {
  memset(&out, 0, sizeof(RoadworksSubCauseCode_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
