#pragma once

#include <etsi_its_denm_coding/SemiAxisLength.h>
#include <etsi_its_denm_conversion/primitives/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/SemiAxisLength.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/semi_axis_length.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_SemiAxisLength(const SemiAxisLength_t& in, denm_msgs::SemiAxisLength& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_SemiAxisLength(const denm_msgs::SemiAxisLength& in, SemiAxisLength_t& out) {

  memset(&out, 0, sizeof(SemiAxisLength_t));
  toStruct_INTEGER(in.value, out);
}

}