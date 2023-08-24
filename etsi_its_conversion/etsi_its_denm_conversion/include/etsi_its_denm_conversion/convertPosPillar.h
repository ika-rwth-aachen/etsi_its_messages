#pragma once

#include <etsi_its_denm_coding/PosPillar.h>
#include <etsi_its_denm_conversion/primitives/convertINTEGER.h>
#ifdef ROS2
#include <etsi_its_denm_msgs/msg/pos_pillar.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#else
#include <etsi_its_denm_msgs/PosPillar.h>
namespace denm_msgs = etsi_its_denm_msgs;
#endif


namespace etsi_its_denm_conversion {

void toRos_PosPillar(const PosPillar_t& in, denm_msgs::PosPillar& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_PosPillar(const denm_msgs::PosPillar& in, PosPillar_t& out) {

  memset(&out, 0, sizeof(PosPillar_t));
  toStruct_INTEGER(in.value, out);
}

}