//// INTEGER HumanPresenceOnTheRoadSubCauseCode


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/HumanPresenceOnTheRoadSubCauseCode.h>
#include <etsi_its_denm_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/HumanPresenceOnTheRoadSubCauseCode.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/human_presence_on_the_road_sub_cause_code.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_HumanPresenceOnTheRoadSubCauseCode(const HumanPresenceOnTheRoadSubCauseCode_t& in, denm_msgs::HumanPresenceOnTheRoadSubCauseCode& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_HumanPresenceOnTheRoadSubCauseCode(const denm_msgs::HumanPresenceOnTheRoadSubCauseCode& in, HumanPresenceOnTheRoadSubCauseCode_t& out) {
  memset(&out, 0, sizeof(HumanPresenceOnTheRoadSubCauseCode_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
