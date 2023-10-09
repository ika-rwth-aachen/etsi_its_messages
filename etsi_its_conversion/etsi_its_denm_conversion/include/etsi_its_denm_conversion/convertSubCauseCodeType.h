#pragma once

#include <etsi_its_denm_coding/SubCauseCodeType.h>
#include <etsi_its_denm_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/SubCauseCodeType.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/sub_cause_code_type.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_SubCauseCodeType(const SubCauseCodeType_t& in, denm_msgs::SubCauseCodeType& out) {

  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_SubCauseCodeType(const denm_msgs::SubCauseCodeType& in, SubCauseCodeType_t& out) {

  memset(&out, 0, sizeof(SubCauseCodeType_t));
  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}