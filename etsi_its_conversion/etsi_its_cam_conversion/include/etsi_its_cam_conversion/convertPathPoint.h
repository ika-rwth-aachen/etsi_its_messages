#pragma once

#include <etsi_its_cam_coding/PathPoint.h>
#include <etsi_its_cam_conversion/convertDeltaReferencePosition.h>
#include <etsi_its_cam_conversion/convertPathDeltaTime.h>
#include <etsi_its_cam_msgs/PathPoint.h>


namespace etsi_its_cam_conversion {

void toRos_PathPoint(const PathPoint_t& in, etsi_its_cam_msgs::PathPoint& out) {

  toRos_DeltaReferencePosition(in.path_position, out.path_position);
  if (in.path_delta_time) {
    toRos_PathDeltaTime(*in.path_delta_time, out.path_delta_time);
    out.path_delta_time_is_present = true;
  }

}

void toStruct_PathPoint(const etsi_its_cam_msgs::PathPoint& in, PathPoint_t& out) {
    
  memset(&out, 0, sizeof(PathPoint_t));

  toStruct_DeltaReferencePosition(in.path_position, out.path_position);
  if (in.path_delta_time_is_present) {
    PathDeltaTime_t path_delta_time;
    toStruct_PathDeltaTime(in.path_delta_time, path_delta_time);
    out.path_delta_time = new PathDeltaTime_t(path_delta_time);
  }

}

}