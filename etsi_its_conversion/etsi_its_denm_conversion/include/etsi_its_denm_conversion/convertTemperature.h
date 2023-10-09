#pragma once

#include <etsi_its_denm_coding/Temperature.h>
#include <etsi_its_denm_conversion/primitives/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/Temperature.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/temperature.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_Temperature(const Temperature_t& in, denm_msgs::Temperature& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_Temperature(const denm_msgs::Temperature& in, Temperature_t& out) {

  memset(&out, 0, sizeof(Temperature_t));
  toStruct_INTEGER(in.value, out);
}

}