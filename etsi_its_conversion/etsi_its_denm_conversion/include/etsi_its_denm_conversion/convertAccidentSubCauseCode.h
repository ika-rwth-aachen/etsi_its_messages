//// INTEGER AccidentSubCauseCode


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/AccidentSubCauseCode.h>
#include <etsi_its_denm_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/AccidentSubCauseCode.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/accident_sub_cause_code.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_AccidentSubCauseCode(const AccidentSubCauseCode_t& in, denm_msgs::AccidentSubCauseCode& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_AccidentSubCauseCode(const denm_msgs::AccidentSubCauseCode& in, AccidentSubCauseCode_t& out) {
  memset(&out, 0, sizeof(AccidentSubCauseCode_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
