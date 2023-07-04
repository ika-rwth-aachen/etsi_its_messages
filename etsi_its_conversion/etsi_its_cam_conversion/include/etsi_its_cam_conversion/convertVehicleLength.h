#pragma once

#include <etsi_its_cam_coding/VehicleLength.h>
#include <etsi_its_cam_conversion/convertVehicleLengthValue.h>
#include <etsi_its_cam_conversion/convertVehicleLengthConfidenceIndication.h>
#include <etsi_its_cam_msgs/VehicleLength.h>


namespace etsi_its_cam_conversion {

void toRos_VehicleLength(const VehicleLength_t& in, etsi_its_cam_msgs::VehicleLength& out) {

  toRos_VehicleLengthValue(in.vehicleLengthValue, out.vehicleLengthValue);
  toRos_VehicleLengthConfidenceIndication(in.vehicleLengthConfidenceIndication, out.vehicleLengthConfidenceIndication);
}

void toStruct_VehicleLength(const etsi_its_cam_msgs::VehicleLength& in, VehicleLength_t& out) {
    
  memset(&out, 0, sizeof(VehicleLength_t));

  toStruct_VehicleLengthValue(in.vehicleLengthValue, out.vehicleLengthValue);
  toStruct_VehicleLengthConfidenceIndication(in.vehicleLengthConfidenceIndication, out.vehicleLengthConfidenceIndication);
}

}