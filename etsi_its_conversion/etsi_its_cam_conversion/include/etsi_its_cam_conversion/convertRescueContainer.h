#pragma once

#include <etsi_its_cam_coding/RescueContainer.h>
#include <etsi_its_cam_conversion/convertLightBarSirenInUse.h>
#include <etsi_its_cam_msgs/RescueContainer.h>


namespace etsi_its_cam_conversion {

void toRos_RescueContainer(const RescueContainer_t& in, etsi_its_cam_msgs::RescueContainer& out) {

  toRos_LightBarSirenInUse(in.lightBarSirenInUse, out.lightBarSirenInUse);
}

void toStruct_RescueContainer(const etsi_its_cam_msgs::RescueContainer& in, RescueContainer_t& out) {
    
  memset(&out, 0, sizeof(RescueContainer_t));

  toStruct_LightBarSirenInUse(in.lightBarSirenInUse, out.lightBarSirenInUse);
}

}