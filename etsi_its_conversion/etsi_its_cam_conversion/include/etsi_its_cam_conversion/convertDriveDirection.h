#pragma once

#include <etsi_its_cam_coding/DriveDirection.h>
#include <etsi_its_cam_msgs/DriveDirection.h>

namespace etsi_its_cam_conversion {
  
void convert_DriveDirectiontoRos(const DriveDirection_t& _DriveDirection_in, etsi_its_cam_msgs::DriveDirection& _DriveDirection_out) {
  _DriveDirection_out.value = _DriveDirection_in;
}

void convert_DriveDirectiontoC(const etsi_its_cam_msgs::DriveDirection& _DriveDirection_in, DriveDirection_t& _DriveDirection_out) {
  memset(&_DriveDirection_out, 0, sizeof(DriveDirection_t));
  _DriveDirection_out = _DriveDirection_in.value;
}

}