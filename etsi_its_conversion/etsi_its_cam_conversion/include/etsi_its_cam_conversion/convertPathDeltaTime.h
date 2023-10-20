#pragma once

#include <etsi_its_cam_coding/PathDeltaTime.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/PathDeltaTime.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/path_delta_time.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_PathDeltaTime(const PathDeltaTime_t& in, cam_msgs::PathDeltaTime& out) {

  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_PathDeltaTime(const cam_msgs::PathDeltaTime& in, PathDeltaTime_t& out) {

  memset(&out, 0, sizeof(PathDeltaTime_t));
  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}