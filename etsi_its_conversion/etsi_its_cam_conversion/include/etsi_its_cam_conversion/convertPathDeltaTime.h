#pragma once

#include <etsi_its_cam_coding/PathDeltaTime.h>
#include <etsi_its_cam_msgs/PathDeltaTime.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion {
  
void toRos_PathDeltaTime(const PathDeltaTime_t& in, etsi_its_cam_msgs::PathDeltaTime& out) {
  toRos_INTEGER(in, out.value);

}

void toStruct_PathDeltaTime(const etsi_its_cam_msgs::PathDeltaTime& in, PathDeltaTime_t& out) {
  memset(&out, 0, sizeof(PathDeltaTime_t));
  toStruct_INTEGER(in.value, out);

}

}