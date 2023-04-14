#pragma once

#include <etsi_its_cam_coding/PathPoint.h>
#include <etsi_its_cam_msgs/PathPoint.h>
#include <etsi_its_cam_conversion/convertDeltaReferencePosition.h>
#include <etsi_its_cam_conversion/convertPathDeltaTime.h>

namespace etsi_its_cam_conversion {
  
void toRos_PathPoint(const PathPoint_t& in, etsi_its_cam_msgs::PathPoint& out) {
  toRos_DeltaReferencePosition(in.pathPosition, out.pathPosition);
  if (in.pathDeltaTime) {
    toRos_PathDeltaTime(*in.pathDeltaTime, out.pathDeltaTime);
    out.pathDeltaTime_isPresent = true;
  }
}

void toStruct_PathPoint(const etsi_its_cam_msgs::PathPoint& in, PathPoint_t& out) {
  memset(&out, 0, sizeof(PathPoint_t));
  toStruct_DeltaReferencePosition(in.pathPosition, out.pathPosition);
  if (in.pathDeltaTime_isPresent) {
    PathDeltaTime_t pathDeltaTime;
    toStruct_PathDeltaTime(in.pathDeltaTime, pathDeltaTime);
    out.pathDeltaTime = new PathDeltaTime_t(pathDeltaTime);
  }
}

}