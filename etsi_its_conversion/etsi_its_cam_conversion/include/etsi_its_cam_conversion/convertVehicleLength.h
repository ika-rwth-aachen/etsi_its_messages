#pragma once

#include <etsi_its_cam_coding/VehicleLength.h>
#include <etsi_its_cam_msgs/VehicleLength.h>
#include <etsi_its_cam_conversion/convertVehicleLengthValue.h>
#include <etsi_its_cam_conversion/convertVehicleLengthConfidenceIndication.h>

namespace etsi_its_cam_conversion
{
	void convert_VehicleLengthtoRos(const VehicleLength_t& _VehicleLength_in, etsi_its_cam_msgs::VehicleLength& _VehicleLength_out)
	{
		convert_VehicleLengthValuetoRos(_VehicleLength_in.vehicleLengthValue, _VehicleLength_out.vehicleLengthValue);
		convert_VehicleLengthConfidenceIndicationtoRos(_VehicleLength_in.vehicleLengthConfidenceIndication, _VehicleLength_out.vehicleLengthConfidenceIndication);
	}
	void convert_VehicleLengthtoC(const etsi_its_cam_msgs::VehicleLength& _VehicleLength_in, VehicleLength_t& _VehicleLength_out)
	{
		memset(&_VehicleLength_out, 0, sizeof(VehicleLength_t));
		convert_VehicleLengthValuetoC(_VehicleLength_in.vehicleLengthValue, _VehicleLength_out.vehicleLengthValue);
		convert_VehicleLengthConfidenceIndicationtoC(_VehicleLength_in.vehicleLengthConfidenceIndication, _VehicleLength_out.vehicleLengthConfidenceIndication);
	}
}