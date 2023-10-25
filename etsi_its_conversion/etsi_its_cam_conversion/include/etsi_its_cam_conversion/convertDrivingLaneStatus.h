#pragma once

#include <etsi_its_cam_coding/DrivingLaneStatus.h>
#include <etsi_its_cam_coding/BIT_STRING.h>
#include <etsi_its_primitives_conversion/convertBIT_STRING.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/DrivingLaneStatus.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/driving_lane_status.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_DrivingLaneStatus(const DrivingLaneStatus_t& in, cam_msgs::DrivingLaneStatus& out) {

  etsi_its_primitives_conversion::toRos_BIT_STRING(in, out.value);
  out.bits_unused = in.bits_unused;
}

void toStruct_DrivingLaneStatus(const cam_msgs::DrivingLaneStatus& in, DrivingLaneStatus_t& out) {

  memset(&out, 0, sizeof(DrivingLaneStatus_t));
  etsi_its_primitives_conversion::toStruct_BIT_STRING(in.value, out);
  out.bits_unused = in.bits_unused;
}

}