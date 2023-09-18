#pragma once

#include <etsi_its_cam_coding/CauseCode.h>
#include <etsi_its_cam_conversion/convertCauseCodeType.h>
#include <etsi_its_cam_conversion/convertSubCauseCodeType.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/CauseCode.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/cause_code.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_CauseCode(const CauseCode_t& in, cam_msgs::CauseCode& out) {

  toRos_CauseCodeType(in.causeCode, out.cause_code);
  toRos_SubCauseCodeType(in.subCauseCode, out.sub_cause_code);
}

void toStruct_CauseCode(const cam_msgs::CauseCode& in, CauseCode_t& out) {

  memset(&out, 0, sizeof(CauseCode_t));

  toStruct_CauseCodeType(in.cause_code, out.causeCode);
  toStruct_SubCauseCodeType(in.sub_cause_code, out.subCauseCode);
}

}