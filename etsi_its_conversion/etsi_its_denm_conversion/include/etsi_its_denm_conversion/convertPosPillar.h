//// INTEGER PosPillar


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/PosPillar.h>
#include <etsi_its_denm_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/PosPillar.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/pos_pillar.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_PosPillar(const PosPillar_t& in, denm_msgs::PosPillar& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_PosPillar(const denm_msgs::PosPillar& in, PosPillar_t& out) {
  memset(&out, 0, sizeof(PosPillar_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
