#pragma once

#include <etsi_its_denm_coding/PathPoint.h>
#include <etsi_its_denm_conversion/convertDeltaReferencePosition.h>
#include <etsi_its_denm_conversion/convertPathDeltaTime.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/PathPoint.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/path_point.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_PathPoint(const PathPoint_t& in, denm_msgs::PathPoint& out) {

  toRos_DeltaReferencePosition(in.pathPosition, out.path_position);
  if (in.pathDeltaTime) {
    toRos_PathDeltaTime(*in.pathDeltaTime, out.path_delta_time);
    out.path_delta_time_is_present = true;
  }

}

void toStruct_PathPoint(const denm_msgs::PathPoint& in, PathPoint_t& out) {

  memset(&out, 0, sizeof(PathPoint_t));

  toStruct_DeltaReferencePosition(in.path_position, out.pathPosition);
  if (in.path_delta_time_is_present) {
    PathDeltaTime_t path_delta_time;
    toStruct_PathDeltaTime(in.path_delta_time, path_delta_time);
    out.pathDeltaTime = new PathDeltaTime_t(path_delta_time);
  }

}

}