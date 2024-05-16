//// INTEGER AccidentSubCauseCode


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/AccidentSubCauseCode.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/AccidentSubCauseCode.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/accident_sub_cause_code.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_AccidentSubCauseCode(const AccidentSubCauseCode_t& in, cam_msgs::AccidentSubCauseCode& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_AccidentSubCauseCode(const cam_msgs::AccidentSubCauseCode& in, AccidentSubCauseCode_t& out) {
  memset(&out, 0, sizeof(AccidentSubCauseCode_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
