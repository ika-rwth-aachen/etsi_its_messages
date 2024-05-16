//// ENUMERATED DriveDirection


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/DriveDirection.h>

#ifdef ROS1
#include <etsi_its_denm_msgs/DriveDirection.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/drive_direction.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_DriveDirection(const DriveDirection_t& in, denm_msgs::DriveDirection& out) {
  out.value = in;
}

void toStruct_DriveDirection(const denm_msgs::DriveDirection& in, DriveDirection_t& out) {
  memset(&out, 0, sizeof(DriveDirection_t));

  out = in.value;
}

}
