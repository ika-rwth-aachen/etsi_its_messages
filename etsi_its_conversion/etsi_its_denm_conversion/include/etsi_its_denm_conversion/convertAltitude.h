//// SEQUENCE Altitude


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/Altitude.h>
#include <etsi_its_denm_conversion/convertAltitudeValue.h>
#include <etsi_its_denm_conversion/convertAltitudeConfidence.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/Altitude.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/altitude.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_Altitude(const Altitude_t& in, denm_msgs::Altitude& out) {
  toRos_AltitudeValue(in.altitudeValue, out.altitude_value);
  toRos_AltitudeConfidence(in.altitudeConfidence, out.altitude_confidence);
}

void toStruct_Altitude(const denm_msgs::Altitude& in, Altitude_t& out) {
  memset(&out, 0, sizeof(Altitude_t));

  toStruct_AltitudeValue(in.altitude_value, out.altitudeValue);
  toStruct_AltitudeConfidence(in.altitude_confidence, out.altitudeConfidence);
}

}
