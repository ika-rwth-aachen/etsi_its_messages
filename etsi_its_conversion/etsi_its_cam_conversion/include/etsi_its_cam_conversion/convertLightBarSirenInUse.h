#pragma once

#include <etsi_its_cam_coding/LightBarSirenInUse.h>
#include <etsi_its_cam_conversion/primitives/convertBIT_STRING.h>
#include <etsi_its_cam_msgs/LightBarSirenInUse.h>


namespace etsi_its_cam_conversion {

void toRos_LightBarSirenInUse(const LightBarSirenInUse_t& in, etsi_its_cam_msgs::LightBarSirenInUse& out) {

  toRos_BIT_STRING(in, out.value);
}

void toStruct_LightBarSirenInUse(const etsi_its_cam_msgs::LightBarSirenInUse& in, LightBarSirenInUse_t& out) {
    
  memset(&out, 0, sizeof(LightBarSirenInUse_t));
  toStruct_BIT_STRING(in.value, out);
}

}