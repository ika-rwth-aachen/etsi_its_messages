#pragma once

#include <etsi_its_denm_coding/DeltaLatitude.h>
#include <etsi_its_denm_conversion/primitives/convertINTEGER.h>
#ifdef ROS2
#include <etsi_its_denm_msgs/msg/delta_latitude.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#else
#include <etsi_its_denm_msgs/DeltaLatitude.h>
namespace denm_msgs = etsi_its_denm_msgs;
#endif


namespace etsi_its_denm_conversion {

void toRos_DeltaLatitude(const DeltaLatitude_t& in, denm_msgs::DeltaLatitude& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_DeltaLatitude(const denm_msgs::DeltaLatitude& in, DeltaLatitude_t& out) {

  memset(&out, 0, sizeof(DeltaLatitude_t));
  toStruct_INTEGER(in.value, out);
}

}