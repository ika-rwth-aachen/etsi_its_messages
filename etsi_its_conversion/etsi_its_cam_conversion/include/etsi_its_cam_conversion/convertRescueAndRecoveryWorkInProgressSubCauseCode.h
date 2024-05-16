//// INTEGER RescueAndRecoveryWorkInProgressSubCauseCode


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/RescueAndRecoveryWorkInProgressSubCauseCode.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/RescueAndRecoveryWorkInProgressSubCauseCode.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/rescue_and_recovery_work_in_progress_sub_cause_code.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_RescueAndRecoveryWorkInProgressSubCauseCode(const RescueAndRecoveryWorkInProgressSubCauseCode_t& in, cam_msgs::RescueAndRecoveryWorkInProgressSubCauseCode& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_RescueAndRecoveryWorkInProgressSubCauseCode(const cam_msgs::RescueAndRecoveryWorkInProgressSubCauseCode& in, RescueAndRecoveryWorkInProgressSubCauseCode_t& out) {
  memset(&out, 0, sizeof(RescueAndRecoveryWorkInProgressSubCauseCode_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
