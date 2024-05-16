//// SEQUENCE-OF PathHistory


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/PathHistory.h>
#include <etsi_its_cam_conversion/convertPathHistory.h>
#include <etsi_its_cam_conversion/convertPathPoint.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/PathHistory.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/path_history.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_PathHistory(const PathHistory_t& in, cam_msgs::PathHistory& out) {
  for (int i = 0; i < in.list.count; ++i) {
    cam_msgs::PathPoint el;
    toRos_PathPoint(*(in.list.array[i]), el);
    out.array.push_back(el);
  }
}

void toStruct_PathHistory(const cam_msgs::PathHistory& in, PathHistory_t& out) {
  memset(&out, 0, sizeof(PathHistory_t));

  for (int i = 0; i < in.array.size(); ++i) {
    PathPoint_t* el = (PathPoint_t*) calloc(1, sizeof(PathPoint_t));
    toStruct_PathPoint(in.array[i], *el);
    if (asn_sequence_add(&out, el)) throw std::invalid_argument("Failed to add to A_SEQUENCE_OF");
  }
}

}
