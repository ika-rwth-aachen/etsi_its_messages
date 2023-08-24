#pragma once

#include <etsi_its_denm_coding/SubCauseCodeType.h>
#include <etsi_its_denm_conversion/primitives/convertINTEGER.h>
#ifdef ROS2
#include <etsi_its_denm_msgs/msg/sub_cause_code_type.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#else
#include <etsi_its_denm_msgs/SubCauseCodeType.h>
namespace denm_msgs = etsi_its_denm_msgs;
#endif


namespace etsi_its_denm_conversion {

void toRos_SubCauseCodeType(const SubCauseCodeType_t& in, denm_msgs::SubCauseCodeType& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_SubCauseCodeType(const denm_msgs::SubCauseCodeType& in, SubCauseCodeType_t& out) {

  memset(&out, 0, sizeof(SubCauseCodeType_t));
  toStruct_INTEGER(in.value, out);
}

}