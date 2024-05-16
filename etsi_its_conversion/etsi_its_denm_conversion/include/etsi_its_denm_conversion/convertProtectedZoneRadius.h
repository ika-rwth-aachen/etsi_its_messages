//// INTEGER ProtectedZoneRadius


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/ProtectedZoneRadius.h>
#include <etsi_its_denm_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/ProtectedZoneRadius.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/protected_zone_radius.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_ProtectedZoneRadius(const ProtectedZoneRadius_t& in, denm_msgs::ProtectedZoneRadius& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_ProtectedZoneRadius(const denm_msgs::ProtectedZoneRadius& in, ProtectedZoneRadius_t& out) {
  memset(&out, 0, sizeof(ProtectedZoneRadius_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
