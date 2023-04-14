#pragma once

#include <etsi_its_cam_coding/DrivingLaneStatus.h>
#include <etsi_its_cam_msgs/DrivingLaneStatus.h>
#include <etsi_its_cam_conversion/primitives/convertBIT_STRING.h>

namespace etsi_its_cam_conversion {
  
void toRos_DrivingLaneStatus(const DrivingLaneStatus_t& in, etsi_its_cam_msgs::DrivingLaneStatus& out) {
  toRos_BIT_STRING(in, out.value);

}

void toStruct_DrivingLaneStatus(const etsi_its_cam_msgs::DrivingLaneStatus& in, DrivingLaneStatus_t& out) {
  memset(&out, 0, sizeof(DrivingLaneStatus_t));
  toStruct_BIT_STRING(in.value, out);

}

}