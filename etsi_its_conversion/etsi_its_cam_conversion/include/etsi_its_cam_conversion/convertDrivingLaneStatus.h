#pragma once

#include <etsi_its_cam_coding/DrivingLaneStatus.h>
#include <etsi_its_cam_conversion/primitives/convertBIT_STRING.h>
#ifdef ROS2
#include <etsi_its_cam_msgs/msg/driving_lane_status.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#else
#include <etsi_its_cam_msgs/DrivingLaneStatus.h>
namespace cam_msgs = etsi_its_cam_msgs;
#endif


namespace etsi_its_cam_conversion {

void toRos_DrivingLaneStatus(const DrivingLaneStatus_t& in, cam_msgs::DrivingLaneStatus& out) {

  toRos_BIT_STRING(in, out.value);
}

void toStruct_DrivingLaneStatus(const cam_msgs::DrivingLaneStatus& in, DrivingLaneStatus_t& out) {
    
  memset(&out, 0, sizeof(DrivingLaneStatus_t));
  toStruct_BIT_STRING(in.value, out);
}

}