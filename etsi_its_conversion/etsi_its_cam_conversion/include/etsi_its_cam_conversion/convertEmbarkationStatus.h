//// BOOLEAN EmbarkationStatus


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/EmbarkationStatus.h>
#include <etsi_its_cam_coding/BOOLEAN.h>
#include <etsi_its_primitives_conversion/convertBOOLEAN.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/EmbarkationStatus.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/embarkation_status.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_EmbarkationStatus(const EmbarkationStatus_t& in, cam_msgs::EmbarkationStatus& out) {
  etsi_its_primitives_conversion::toRos_BOOLEAN(in, out.value);
}

void toStruct_EmbarkationStatus(const cam_msgs::EmbarkationStatus& in, EmbarkationStatus_t& out) {
  memset(&out, 0, sizeof(EmbarkationStatus_t));

  etsi_its_primitives_conversion::toStruct_BOOLEAN(in.value, out);
}

}
