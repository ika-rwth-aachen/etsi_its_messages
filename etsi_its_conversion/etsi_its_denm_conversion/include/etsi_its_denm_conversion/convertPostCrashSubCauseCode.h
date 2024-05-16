//// INTEGER PostCrashSubCauseCode


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/PostCrashSubCauseCode.h>
#include <etsi_its_denm_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/PostCrashSubCauseCode.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/post_crash_sub_cause_code.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_PostCrashSubCauseCode(const PostCrashSubCauseCode_t& in, denm_msgs::PostCrashSubCauseCode& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_PostCrashSubCauseCode(const denm_msgs::PostCrashSubCauseCode& in, PostCrashSubCauseCode_t& out) {
  memset(&out, 0, sizeof(PostCrashSubCauseCode_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
