#pragma once

#include <etsi_its_denm_coding/TurningRadius.h>
#include <etsi_its_denm_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/TurningRadius.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/turning_radius.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_TurningRadius(const TurningRadius_t& in, denm_msgs::TurningRadius& out) {

  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_TurningRadius(const denm_msgs::TurningRadius& in, TurningRadius_t& out) {

  memset(&out, 0, sizeof(TurningRadius_t));
  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}