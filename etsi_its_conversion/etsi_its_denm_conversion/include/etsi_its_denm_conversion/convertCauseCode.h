#pragma once

#include <etsi_its_denm_coding/CauseCode.h>
#include <etsi_its_denm_conversion/convertCauseCodeType.h>
#include <etsi_its_denm_conversion/convertSubCauseCodeType.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/CauseCode.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/cause_code.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_CauseCode(const CauseCode_t& in, denm_msgs::CauseCode& out) {

  toRos_CauseCodeType(in.causeCode, out.cause_code);
  toRos_SubCauseCodeType(in.subCauseCode, out.sub_cause_code);
}

void toStruct_CauseCode(const denm_msgs::CauseCode& in, CauseCode_t& out) {

  memset(&out, 0, sizeof(CauseCode_t));

  toStruct_CauseCodeType(in.cause_code, out.causeCode);
  toStruct_SubCauseCodeType(in.sub_cause_code, out.subCauseCode);
}

}