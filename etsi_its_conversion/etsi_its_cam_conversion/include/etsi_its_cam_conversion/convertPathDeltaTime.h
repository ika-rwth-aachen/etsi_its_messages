#pragma once

#include <etsi_its_cam_coding/PathDeltaTime.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>
#ifdef ROS2
#include <etsi_its_cam_msgs/msg/path_delta_time.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#else
#include <etsi_its_cam_msgs/PathDeltaTime.h>
namespace cam_msgs = etsi_its_cam_msgs;
#endif


namespace etsi_its_cam_conversion {

void toRos_PathDeltaTime(const PathDeltaTime_t& in, cam_msgs::PathDeltaTime& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_PathDeltaTime(const cam_msgs::PathDeltaTime& in, PathDeltaTime_t& out) {

  memset(&out, 0, sizeof(PathDeltaTime_t));
  toStruct_INTEGER(in.value, out);
}

}