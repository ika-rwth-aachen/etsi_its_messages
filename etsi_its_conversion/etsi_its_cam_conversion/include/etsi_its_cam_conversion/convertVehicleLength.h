#pragma once

#include <etsi_its_cam_coding/VehicleLength.h>
#include <etsi_its_cam_conversion/convertVehicleLengthValue.h>
#include <etsi_its_cam_conversion/convertVehicleLengthConfidenceIndication.h>
#include <etsi_its_cam_msgs/VehicleLength.h>


namespace etsi_its_cam_conversion {

void toRos_VehicleLength(const VehicleLength_t& in, etsi_its_cam_msgs::VehicleLength& out) {

  toRos_VehicleLengthValue(in.vehicleLengthValue, out.vehicle_length_value);
  toRos_VehicleLengthConfidenceIndication(in.vehicleLengthConfidenceIndication, out.vehicle_length_confidence_indication);
}

void toStruct_VehicleLength(const etsi_its_cam_msgs::VehicleLength& in, VehicleLength_t& out) {
    
  memset(&out, 0, sizeof(VehicleLength_t));

  toStruct_VehicleLengthValue(in.vehicle_length_value, out.vehicleLengthValue);
  toStruct_VehicleLengthConfidenceIndication(in.vehicle_length_confidence_indication, out.vehicleLengthConfidenceIndication);
}

}