//// INTEGER CauseCodeType


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/CauseCodeType.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/CauseCodeType.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/cause_code_type.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_CauseCodeType(const CauseCodeType_t& in, cam_msgs::CauseCodeType& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_CauseCodeType(const cam_msgs::CauseCodeType& in, CauseCodeType_t& out) {
  memset(&out, 0, sizeof(CauseCodeType_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
