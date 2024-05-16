//// INTEGER DangerousSituationSubCauseCode


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/DangerousSituationSubCauseCode.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/DangerousSituationSubCauseCode.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/dangerous_situation_sub_cause_code.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_DangerousSituationSubCauseCode(const DangerousSituationSubCauseCode_t& in, cam_msgs::DangerousSituationSubCauseCode& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_DangerousSituationSubCauseCode(const cam_msgs::DangerousSituationSubCauseCode& in, DangerousSituationSubCauseCode_t& out) {
  memset(&out, 0, sizeof(DangerousSituationSubCauseCode_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
