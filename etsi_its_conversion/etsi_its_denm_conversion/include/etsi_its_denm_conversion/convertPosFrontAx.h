#pragma once

#include <etsi_its_denm_coding/PosFrontAx.h>
#include <etsi_its_denm_conversion/primitives/convertINTEGER.h>
#ifdef ROS2
#include <etsi_its_denm_msgs/msg/pos_front_ax.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#else
#include <etsi_its_denm_msgs/PosFrontAx.h>
namespace denm_msgs = etsi_its_denm_msgs;
#endif


namespace etsi_its_denm_conversion {

void toRos_PosFrontAx(const PosFrontAx_t& in, denm_msgs::PosFrontAx& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_PosFrontAx(const denm_msgs::PosFrontAx& in, PosFrontAx_t& out) {

  memset(&out, 0, sizeof(PosFrontAx_t));
  toStruct_INTEGER(in.value, out);
}

}