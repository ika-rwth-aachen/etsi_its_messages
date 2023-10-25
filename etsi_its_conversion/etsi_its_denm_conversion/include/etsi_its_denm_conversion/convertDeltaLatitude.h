#pragma once

#include <etsi_its_denm_coding/DeltaLatitude.h>
#include <etsi_its_denm_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/DeltaLatitude.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/delta_latitude.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_DeltaLatitude(const DeltaLatitude_t& in, denm_msgs::DeltaLatitude& out) {

  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_DeltaLatitude(const denm_msgs::DeltaLatitude& in, DeltaLatitude_t& out) {

  memset(&out, 0, sizeof(DeltaLatitude_t));
  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}