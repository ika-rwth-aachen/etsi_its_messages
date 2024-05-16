//// ENUMERATED VehicleLengthConfidenceIndication


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/VehicleLengthConfidenceIndication.h>

#ifdef ROS1
#include <etsi_its_denm_msgs/VehicleLengthConfidenceIndication.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/vehicle_length_confidence_indication.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_VehicleLengthConfidenceIndication(const VehicleLengthConfidenceIndication_t& in, denm_msgs::VehicleLengthConfidenceIndication& out) {
  out.value = in;
}

void toStruct_VehicleLengthConfidenceIndication(const denm_msgs::VehicleLengthConfidenceIndication& in, VehicleLengthConfidenceIndication_t& out) {
  memset(&out, 0, sizeof(VehicleLengthConfidenceIndication_t));

  out = in.value;
}

}
