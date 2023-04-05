#pragma once

#include <etsi_its_cam_coding/VehicleLength.h>
#include <etsi_its_cam_msgs/VehicleLength.h>
#include <etsi_its_cam_conversion/convertVehicleLengthValue.h>
#include <etsi_its_cam_conversion/convertVehicleLengthConfidenceIndication.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::VehicleLength convert_VehicleLengthtoRos(const VehicleLength_t& _VehicleLength_in)
	{
		etsi_its_cam_msgs::VehicleLength VehicleLength_out;
		VehicleLength_out.vehicleLengthValue = convert_VehicleLengthValuetoRos(_VehicleLength_in.vehicleLengthValue);
		VehicleLength_out.vehicleLengthConfidenceIndication = convert_VehicleLengthConfidenceIndicationtoRos(_VehicleLength_in.vehicleLengthConfidenceIndication);
		return VehicleLength_out;
	}
	VehicleLength_t convert_VehicleLengthtoC(const etsi_its_cam_msgs::VehicleLength& _VehicleLength_in)
	{
		VehicleLength_t VehicleLength_out;
		VehicleLength_out.vehicleLengthValue = convert_VehicleLengthValuetoC(_VehicleLength_in.vehicleLengthValue);
		VehicleLength_out.vehicleLengthConfidenceIndication = convert_VehicleLengthConfidenceIndicationtoC(_VehicleLength_in.vehicleLengthConfidenceIndication);
		return VehicleLength_out;
	}
}