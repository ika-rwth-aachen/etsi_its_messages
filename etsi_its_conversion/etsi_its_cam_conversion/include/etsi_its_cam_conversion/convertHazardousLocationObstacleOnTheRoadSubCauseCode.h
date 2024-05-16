//// INTEGER HazardousLocationObstacleOnTheRoadSubCauseCode


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/HazardousLocation-ObstacleOnTheRoadSubCauseCode.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/HazardousLocationObstacleOnTheRoadSubCauseCode.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/hazardous_location_obstacle_on_the_road_sub_cause_code.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_HazardousLocationObstacleOnTheRoadSubCauseCode(const HazardousLocation_ObstacleOnTheRoadSubCauseCode_t& in, cam_msgs::HazardousLocationObstacleOnTheRoadSubCauseCode& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_HazardousLocationObstacleOnTheRoadSubCauseCode(const cam_msgs::HazardousLocationObstacleOnTheRoadSubCauseCode& in, HazardousLocation_ObstacleOnTheRoadSubCauseCode_t& out) {
  memset(&out, 0, sizeof(HazardousLocation_ObstacleOnTheRoadSubCauseCode_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
