//// INTEGER TrafficConditionSubCauseCode


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/TrafficConditionSubCauseCode.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/TrafficConditionSubCauseCode.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/traffic_condition_sub_cause_code.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_TrafficConditionSubCauseCode(const TrafficConditionSubCauseCode_t& in, cam_msgs::TrafficConditionSubCauseCode& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_TrafficConditionSubCauseCode(const cam_msgs::TrafficConditionSubCauseCode& in, TrafficConditionSubCauseCode_t& out) {
  memset(&out, 0, sizeof(TrafficConditionSubCauseCode_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
