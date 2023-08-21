#pragma once

#include <etsi_its_cam_coding/CauseCodeType.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>
#ifdef ROS2
#include <etsi_its_cam_msgs/msg/cause_code_type.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#else
#include <etsi_its_cam_msgs/CauseCodeType.h>
namespace cam_msgs = etsi_its_cam_msgs;
#endif


namespace etsi_its_cam_conversion {

void toRos_CauseCodeType(const CauseCodeType_t& in, cam_msgs::CauseCodeType& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_CauseCodeType(const cam_msgs::CauseCodeType& in, CauseCodeType_t& out) {
    
  memset(&out, 0, sizeof(CauseCodeType_t));
  toStruct_INTEGER(in.value, out);
}

}