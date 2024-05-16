//// INTEGER StationType


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/StationType.h>
#include <etsi_its_denm_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/StationType.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/station_type.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_StationType(const StationType_t& in, denm_msgs::StationType& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_StationType(const denm_msgs::StationType& in, StationType_t& out) {
  memset(&out, 0, sizeof(StationType_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
