#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/asn_SEQUENCE_OF.h>
#include <etsi_its_denm_coding/PathHistory.h>
#include <etsi_its_denm_coding/PathPoint.h>
#include <etsi_its_denm_conversion/convertPathPoint.h>
#ifdef ROS2
#include <etsi_its_denm_msgs/msg/path_point.hpp>
#include <etsi_its_denm_msgs/msg/path_history.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#else
#include <etsi_its_denm_msgs/PathPoint.h>
#include <etsi_its_denm_msgs/PathHistory.h>
namespace denm_msgs = etsi_its_denm_msgs;
#endif


namespace etsi_its_denm_conversion {

void toRos_PathHistory(const PathHistory_t& in, denm_msgs::PathHistory& out) {

  for (int i = 0; i < in.list.count; i++) {
    denm_msgs::PathPoint array;
    toRos_PathPoint(*(in.list.array[i]), array);
    out.array.push_back(array);
  }

}

void toStruct_PathHistory(const denm_msgs::PathHistory& in, PathHistory_t& out) {
    
  memset(&out, 0, sizeof(PathHistory_t));

  for (int i = 0; i < in.array.size(); i++) {
    PathPoint_t array;
    toStruct_PathPoint(in.array[i], array);
    PathPoint_t* array_ptr = new PathPoint_t(array);
    int status = asn_sequence_add(&out, array_ptr);
    if (status != 0) throw std::invalid_argument("Failed to add to A_SEQUENCE_OF");
  }

}

}