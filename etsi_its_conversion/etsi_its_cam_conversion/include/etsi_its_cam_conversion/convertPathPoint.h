#pragma once

#include <etsi_its_cam_coding/PathPoint.h>
#include <etsi_its_cam_msgs/PathPoint.h>
#include <etsi_its_cam_conversion/convertDeltaReferencePosition.h>
#include <etsi_its_cam_conversion/convertPathDeltaTime.h>

namespace etsi_its_cam_conversion {
  
void convert_PathPointtoRos(const PathPoint_t& _PathPoint_in, etsi_its_cam_msgs::PathPoint& _PathPoint_out) {
  convert_DeltaReferencePositiontoRos(_PathPoint_in.pathPosition, _PathPoint_out.pathPosition);
  if (_PathPoint_in.pathDeltaTime) {
    convert_PathDeltaTimetoRos(*_PathPoint_in.pathDeltaTime, _PathPoint_out.pathDeltaTime);
    _PathPoint_out.pathDeltaTime_isPresent = true;
  }
}

void convert_PathPointtoC(const etsi_its_cam_msgs::PathPoint& _PathPoint_in, PathPoint_t& _PathPoint_out) {
  memset(&_PathPoint_out, 0, sizeof(PathPoint_t));
  convert_DeltaReferencePositiontoC(_PathPoint_in.pathPosition, _PathPoint_out.pathPosition);
  if (_PathPoint_in.pathDeltaTime_isPresent) {
    PathDeltaTime_t pathDeltaTime;
    convert_PathDeltaTimetoC(_PathPoint_in.pathDeltaTime, pathDeltaTime);
    _PathPoint_out.pathDeltaTime = new PathDeltaTime_t(pathDeltaTime);
  }
}

}