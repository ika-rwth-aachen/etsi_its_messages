#pragma once

#include <etsi_its_cam_coding/RescueContainer.h>
#include <etsi_its_cam_msgs/RescueContainer.h>
#include <etsi_its_cam_conversion/convertLightBarSirenInUse.h>

namespace etsi_its_cam_conversion {
  
void convert_RescueContainertoRos(const RescueContainer_t& _RescueContainer_in, etsi_its_cam_msgs::RescueContainer& _RescueContainer_out) {
  convert_LightBarSirenInUsetoRos(_RescueContainer_in.lightBarSirenInUse, _RescueContainer_out.lightBarSirenInUse);
}

void convert_RescueContainertoC(const etsi_its_cam_msgs::RescueContainer& _RescueContainer_in, RescueContainer_t& _RescueContainer_out) {
  memset(&_RescueContainer_out, 0, sizeof(RescueContainer_t));
  convert_LightBarSirenInUsetoC(_RescueContainer_in.lightBarSirenInUse, _RescueContainer_out.lightBarSirenInUse);
}

}