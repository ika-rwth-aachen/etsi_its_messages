//// INTEGER AltitudeValue


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/AltitudeValue.h>
#include <etsi_its_denm_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/AltitudeValue.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/altitude_value.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_AltitudeValue(const AltitudeValue_t& in, denm_msgs::AltitudeValue& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_AltitudeValue(const denm_msgs::AltitudeValue& in, AltitudeValue_t& out) {
  memset(&out, 0, sizeof(AltitudeValue_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
