//// INTEGER SignalViolationSubCauseCode


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/SignalViolationSubCauseCode.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/SignalViolationSubCauseCode.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/signal_violation_sub_cause_code.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_SignalViolationSubCauseCode(const SignalViolationSubCauseCode_t& in, cam_msgs::SignalViolationSubCauseCode& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_SignalViolationSubCauseCode(const cam_msgs::SignalViolationSubCauseCode& in, SignalViolationSubCauseCode_t& out) {
  memset(&out, 0, sizeof(SignalViolationSubCauseCode_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
