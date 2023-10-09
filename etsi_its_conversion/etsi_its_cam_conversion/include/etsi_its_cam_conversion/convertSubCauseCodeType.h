#pragma once

#include <etsi_its_cam_coding/SubCauseCodeType.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/SubCauseCodeType.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/sub_cause_code_type.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_SubCauseCodeType(const SubCauseCodeType_t& in, cam_msgs::SubCauseCodeType& out) {

  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_SubCauseCodeType(const cam_msgs::SubCauseCodeType& in, SubCauseCodeType_t& out) {

  memset(&out, 0, sizeof(SubCauseCodeType_t));
  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}