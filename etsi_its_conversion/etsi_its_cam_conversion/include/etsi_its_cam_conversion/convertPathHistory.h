#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/asn_SEQUENCE_OF.h>
#include <etsi_its_cam_coding/PathHistory.h>
#include <etsi_its_cam_coding/PathPoint.h>
#include <etsi_its_cam_conversion/convertPathPoint.h>
#ifdef ROS2
#include <etsi_its_cam_msgs/msg/path_point.hpp>
#include <etsi_its_cam_msgs/msg/path_history.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#else
#include <etsi_its_cam_msgs/PathPoint.h>
#include <etsi_its_cam_msgs/PathHistory.h>
namespace cam_msgs = etsi_its_cam_msgs;
#endif


namespace etsi_its_cam_conversion {

void toRos_PathHistory(const PathHistory_t& in, cam_msgs::PathHistory& out) {

  for (int i = 0; i < in.list.count; i++) {
    cam_msgs::PathPoint array;
    toRos_PathPoint(*(in.list.array[i]), array);
    out.array.push_back(array);
  }

}

void toStruct_PathHistory(const cam_msgs::PathHistory& in, PathHistory_t& out) {
    
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