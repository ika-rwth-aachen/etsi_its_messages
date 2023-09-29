#pragma once

#include <etsi_its_cam_coding/RescueContainer.h>
#include <etsi_its_cam_conversion/convertLightBarSirenInUse.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/RescueContainer.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/rescue_container.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_RescueContainer(const RescueContainer_t& in, cam_msgs::RescueContainer& out) {

  toRos_LightBarSirenInUse(in.lightBarSirenInUse, out.light_bar_siren_in_use);
}

void toStruct_RescueContainer(const cam_msgs::RescueContainer& in, RescueContainer_t& out) {

  memset(&out, 0, sizeof(RescueContainer_t));

  toStruct_LightBarSirenInUse(in.light_bar_siren_in_use, out.lightBarSirenInUse);
}

}