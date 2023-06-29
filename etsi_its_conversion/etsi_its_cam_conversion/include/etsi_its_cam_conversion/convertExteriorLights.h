#pragma once

#include <etsi_its_cam_coding/ExteriorLights.h>
#include <etsi_its_cam_conversion/primitives/convertBIT_STRING.h>
#include <etsi_its_cam_msgs/ExteriorLights.h>


namespace etsi_its_cam_conversion {

void toRos_ExteriorLights(const ExteriorLights_t& in, etsi_its_cam_msgs::ExteriorLights& out) {

  toRos_BIT_STRING(in, out.value);
}

void toStruct_ExteriorLights(const etsi_its_cam_msgs::ExteriorLights& in, ExteriorLights_t& out) {
    
  memset(&out, 0, sizeof(ExteriorLights_t));
  toStruct_BIT_STRING(in.value, out);
}

}