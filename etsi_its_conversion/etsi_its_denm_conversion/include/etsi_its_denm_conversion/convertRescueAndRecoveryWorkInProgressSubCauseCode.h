//// INTEGER RescueAndRecoveryWorkInProgressSubCauseCode


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/RescueAndRecoveryWorkInProgressSubCauseCode.h>
#include <etsi_its_denm_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/RescueAndRecoveryWorkInProgressSubCauseCode.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/rescue_and_recovery_work_in_progress_sub_cause_code.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_RescueAndRecoveryWorkInProgressSubCauseCode(const RescueAndRecoveryWorkInProgressSubCauseCode_t& in, denm_msgs::RescueAndRecoveryWorkInProgressSubCauseCode& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_RescueAndRecoveryWorkInProgressSubCauseCode(const denm_msgs::RescueAndRecoveryWorkInProgressSubCauseCode& in, RescueAndRecoveryWorkInProgressSubCauseCode_t& out) {
  memset(&out, 0, sizeof(RescueAndRecoveryWorkInProgressSubCauseCode_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
