#pragma once

#include <etsi_its_cam_coding/DriveDirection.h>
#include <etsi_its_cam_msgs/DriveDirection.h>


namespace etsi_its_cam_conversion {

void toRos_DriveDirection(const DriveDirection_t& in, etsi_its_cam_msgs::DriveDirection& out) {

  out.value = in;
}

void toStruct_DriveDirection(const etsi_its_cam_msgs::DriveDirection& in, DriveDirection_t& out) {
    
  memset(&out, 0, sizeof(DriveDirection_t));
  out = in.value;
}

}