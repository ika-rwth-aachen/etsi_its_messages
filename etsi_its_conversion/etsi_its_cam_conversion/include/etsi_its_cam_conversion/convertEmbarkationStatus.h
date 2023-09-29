#pragma once

#include <etsi_its_cam_coding/EmbarkationStatus.h>
#include <etsi_its_cam_conversion/primitives/convertBOOLEAN.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/EmbarkationStatus.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/embarkation_status.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_EmbarkationStatus(const EmbarkationStatus_t& in, cam_msgs::EmbarkationStatus& out) {

  toRos_BOOLEAN(in, out.value);
}

void toStruct_EmbarkationStatus(const cam_msgs::EmbarkationStatus& in, EmbarkationStatus_t& out) {

  memset(&out, 0, sizeof(EmbarkationStatus_t));
  toStruct_BOOLEAN(in.value, out);
}

}