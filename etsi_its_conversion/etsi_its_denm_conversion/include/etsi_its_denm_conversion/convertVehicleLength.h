//// SEQUENCE VehicleLength


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/VehicleLength.h>
#include <etsi_its_denm_conversion/convertVehicleLengthValue.h>
#include <etsi_its_denm_conversion/convertVehicleLengthConfidenceIndication.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/VehicleLength.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/vehicle_length.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_VehicleLength(const VehicleLength_t& in, denm_msgs::VehicleLength& out) {
  toRos_VehicleLengthValue(in.vehicleLengthValue, out.vehicle_length_value);
  toRos_VehicleLengthConfidenceIndication(in.vehicleLengthConfidenceIndication, out.vehicle_length_confidence_indication);
}

void toStruct_VehicleLength(const denm_msgs::VehicleLength& in, VehicleLength_t& out) {
  memset(&out, 0, sizeof(VehicleLength_t));

  toStruct_VehicleLengthValue(in.vehicle_length_value, out.vehicleLengthValue);
  toStruct_VehicleLengthConfidenceIndication(in.vehicle_length_confidence_indication, out.vehicleLengthConfidenceIndication);
}

}
