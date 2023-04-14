#pragma once

#include <etsi_its_cam_coding/DrivingLaneStatus.h>
#include <etsi_its_cam_msgs/DrivingLaneStatus.h>
#include <etsi_its_cam_conversion/primitives/convertBIT_STRING.h>

namespace etsi_its_cam_conversion {
  
void convert_DrivingLaneStatustoRos(const DrivingLaneStatus_t& _DrivingLaneStatus_in, etsi_its_cam_msgs::DrivingLaneStatus& _DrivingLaneStatus_out) {
  convert_BIT_STRINGtoRos(_DrivingLaneStatus_in, _DrivingLaneStatus_out.value);

}

void convert_DrivingLaneStatustoC(const etsi_its_cam_msgs::DrivingLaneStatus& _DrivingLaneStatus_in, DrivingLaneStatus_t& _DrivingLaneStatus_out) {
  memset(&_DrivingLaneStatus_out, 0, sizeof(DrivingLaneStatus_t));
  convert_BIT_STRINGtoC(_DrivingLaneStatus_in.value, _DrivingLaneStatus_out);

}

}