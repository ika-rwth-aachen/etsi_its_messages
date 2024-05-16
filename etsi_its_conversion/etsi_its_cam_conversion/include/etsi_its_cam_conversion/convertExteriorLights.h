//// BIT-STRING ExteriorLights


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/ExteriorLights.h>
#include <etsi_its_cam_coding/BIT_STRING.h>
#include <etsi_its_primitives_conversion/convertBIT_STRING.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/ExteriorLights.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/exterior_lights.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_ExteriorLights(const ExteriorLights_t& in, cam_msgs::ExteriorLights& out) {
  etsi_its_primitives_conversion::toRos_BIT_STRING(in, out.value);
  out.bits_unused = in.bits_unused;
}

void toStruct_ExteriorLights(const cam_msgs::ExteriorLights& in, ExteriorLights_t& out) {
  memset(&out, 0, sizeof(ExteriorLights_t));

  etsi_its_primitives_conversion::toStruct_BIT_STRING(in.value, out);
  out.bits_unused = in.bits_unused;
}

}
