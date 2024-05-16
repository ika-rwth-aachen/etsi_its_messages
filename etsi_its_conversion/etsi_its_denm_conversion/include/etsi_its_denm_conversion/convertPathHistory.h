//// SEQUENCE-OF PathHistory


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/PathHistory.h>
#include <etsi_its_denm_conversion/convertPathHistory.h>
#include <etsi_its_denm_conversion/convertPathPoint.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/PathHistory.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/path_history.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_PathHistory(const PathHistory_t& in, denm_msgs::PathHistory& out) {
  for (int i = 0; i < in.list.count; ++i) {
    denm_msgs::PathPoint el;
    toRos_PathPoint(*(in.list.array[i]), el);
    out.array.push_back(el);
  }
}

void toStruct_PathHistory(const denm_msgs::PathHistory& in, PathHistory_t& out) {
  memset(&out, 0, sizeof(PathHistory_t));

  for (int i = 0; i < in.array.size(); ++i) {
    PathPoint_t* el = (PathPoint_t*) calloc(1, sizeof(PathPoint_t));
    toStruct_PathPoint(in.array[i], *el);
    if (asn_sequence_add(&out, el)) throw std::invalid_argument("Failed to add to A_SEQUENCE_OF");
  }
}

}
