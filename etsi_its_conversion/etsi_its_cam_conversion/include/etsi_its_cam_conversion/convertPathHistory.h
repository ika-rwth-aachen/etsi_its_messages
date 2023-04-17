#pragma once

#include <etsi_its_cam_coding/PathHistory.h>
#include <etsi_its_cam_msgs/PathHistory.h>
#include <etsi_its_cam_coding/PathPoint.h>
#include <etsi_its_cam_msgs/PathPoint.h>
#include <etsi_its_cam_conversion/convertPathPoint.h>

#include <stdexcept>
#include <etsi_its_cam_coding/asn_SEQUENCE_OF.h>

namespace etsi_its_cam_conversion {
  
void toRos_PathHistory(const PathHistory_t& in, etsi_its_cam_msgs::PathHistory& out) {
  for (int i = 0; i < in.list.count; i++) {
    etsi_its_cam_msgs::PathPoint pathPoint;
    toRos_PathPoint(*(in.list.array[i]), pathPoint);
    out.array.push_back(pathPoint);
  }
}

void toStruct_PathHistory(const etsi_its_cam_msgs::PathHistory& in, PathHistory_t& out) {
  memset(&out, 0, sizeof(PathHistory_t));
  for (int i = 0; i < in.array.size(); i++) {
    PathPoint_t pathPoint;
    toStruct_PathPoint(in.array[i], pathPoint);
    PathPoint_t* pathPoint_ptr = new PathPoint_t(pathPoint);
    int status = asn_sequence_add(&out, pathPoint_ptr);
    if (status != 0) throw std::invalid_argument("Failed to add to A_SEQUENCE_OF");
  }
}

}