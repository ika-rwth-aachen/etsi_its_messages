#pragma once

#include <etsi_its_cam_coding/PathHistory.h>
#include <etsi_its_cam_msgs/PathHistory.h>

namespace etsi_its_cam_conversion {
  
void toRos_PathHistory(const PathHistory_t& in, etsi_its_cam_msgs::PathHistory& out) {
}

void toStruct_PathHistory(const etsi_its_cam_msgs::PathHistory& in, PathHistory_t& out) {
  memset(&out, 0, sizeof(PathHistory_t));
}

}