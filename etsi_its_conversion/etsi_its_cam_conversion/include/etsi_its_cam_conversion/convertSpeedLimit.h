#pragma once

#include <etsi_its_cam_coding/SpeedLimit.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>
#ifdef ROS2
#include <etsi_its_cam_msgs/msg/speed_limit.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#else
#include <etsi_its_cam_msgs/SpeedLimit.h>
namespace cam_msgs = etsi_its_cam_msgs;
#endif


namespace etsi_its_cam_conversion {

void toRos_SpeedLimit(const SpeedLimit_t& in, cam_msgs::SpeedLimit& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_SpeedLimit(const cam_msgs::SpeedLimit& in, SpeedLimit_t& out) {
    
  memset(&out, 0, sizeof(SpeedLimit_t));
  toStruct_INTEGER(in.value, out);
}

}