//// INTEGER HumanProblemSubCauseCode


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/HumanProblemSubCauseCode.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/HumanProblemSubCauseCode.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/human_problem_sub_cause_code.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_HumanProblemSubCauseCode(const HumanProblemSubCauseCode_t& in, cam_msgs::HumanProblemSubCauseCode& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_HumanProblemSubCauseCode(const cam_msgs::HumanProblemSubCauseCode& in, HumanProblemSubCauseCode_t& out) {
  memset(&out, 0, sizeof(HumanProblemSubCauseCode_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
