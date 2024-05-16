//// BIT-STRING LightBarSirenInUse


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/LightBarSirenInUse.h>
#include <etsi_its_denm_coding/BIT_STRING.h>
#include <etsi_its_primitives_conversion/convertBIT_STRING.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/LightBarSirenInUse.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/light_bar_siren_in_use.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_LightBarSirenInUse(const LightBarSirenInUse_t& in, denm_msgs::LightBarSirenInUse& out) {
  etsi_its_primitives_conversion::toRos_BIT_STRING(in, out.value);
  out.bits_unused = in.bits_unused;
}

void toStruct_LightBarSirenInUse(const denm_msgs::LightBarSirenInUse& in, LightBarSirenInUse_t& out) {
  memset(&out, 0, sizeof(LightBarSirenInUse_t));

  etsi_its_primitives_conversion::toStruct_BIT_STRING(in.value, out);
  out.bits_unused = in.bits_unused;
}

}
