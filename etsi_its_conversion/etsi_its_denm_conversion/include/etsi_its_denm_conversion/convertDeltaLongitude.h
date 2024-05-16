//// INTEGER DeltaLongitude


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/DeltaLongitude.h>
#include <etsi_its_denm_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/DeltaLongitude.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/delta_longitude.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_DeltaLongitude(const DeltaLongitude_t& in, denm_msgs::DeltaLongitude& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_DeltaLongitude(const denm_msgs::DeltaLongitude& in, DeltaLongitude_t& out) {
  memset(&out, 0, sizeof(DeltaLongitude_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
