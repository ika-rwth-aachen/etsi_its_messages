#pragma once

#include <etsi_its_cam_coding/VehicleLengthConfidenceIndication.h>
#include <etsi_its_cam_msgs/VehicleLengthConfidenceIndication.h>

namespace etsi_its_cam_conversion
{
	void convert_VehicleLengthConfidenceIndicationtoRos(const VehicleLengthConfidenceIndication_t& _VehicleLengthConfidenceIndication_in, etsi_its_cam_msgs::VehicleLengthConfidenceIndication& _VehicleLengthConfidenceIndication_out)
	{
		_VehicleLengthConfidenceIndication_out.value = _VehicleLengthConfidenceIndication_in;
	}
	void convert_VehicleLengthConfidenceIndicationtoC(const etsi_its_cam_msgs::VehicleLengthConfidenceIndication& _VehicleLengthConfidenceIndication_in, VehicleLengthConfidenceIndication_t& _VehicleLengthConfidenceIndication_out)
	{
		memset(&_VehicleLengthConfidenceIndication_out, 0, sizeof(VehicleLengthConfidenceIndication_t));
		_VehicleLengthConfidenceIndication_out = _VehicleLengthConfidenceIndication_in.value;
	}
}