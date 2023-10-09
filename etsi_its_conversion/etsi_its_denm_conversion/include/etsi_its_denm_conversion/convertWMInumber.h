#pragma once

#include <etsi_its_denm_coding/WMInumber.h>
#include <etsi_its_denm_conversion/primitives/convertIA5String.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/WMInumber.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/wm_inumber.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_WMInumber(const WMInumber_t& in, denm_msgs::WMInumber& out) {

  toRos_IA5String(in, out.value);
}

void toStruct_WMInumber(const denm_msgs::WMInumber& in, WMInumber_t& out) {

  memset(&out, 0, sizeof(WMInumber_t));
  toStruct_IA5String(in.value, out);
}

}