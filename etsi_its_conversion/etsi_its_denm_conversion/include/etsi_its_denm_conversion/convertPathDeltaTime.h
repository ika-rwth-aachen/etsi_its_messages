#pragma once

#include <etsi_its_denm_coding/PathDeltaTime.h>
#include <etsi_its_denm_conversion/primitives/convertINTEGER.h>
#ifdef ROS2
#include <etsi_its_denm_msgs/msg/path_delta_time.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#else
#include <etsi_its_denm_msgs/PathDeltaTime.h>
namespace denm_msgs = etsi_its_denm_msgs;
#endif


namespace etsi_its_denm_conversion {

void toRos_PathDeltaTime(const PathDeltaTime_t& in, denm_msgs::PathDeltaTime& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_PathDeltaTime(const denm_msgs::PathDeltaTime& in, PathDeltaTime_t& out) {

  memset(&out, 0, sizeof(PathDeltaTime_t));
  toStruct_INTEGER(in.value, out);
}

}