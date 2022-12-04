#pragma once

#include <VehicleLengthConfidenceIndication.h>
#include <etsi_its_cam_msgs/VehicleLengthConfidenceIndication.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::VehicleLengthConfidenceIndication convert_VehicleLengthConfidenceIndicationtoRos(const VehicleLengthConfidenceIndication_t& _VehicleLengthConfidenceIndication_in)
	{
		etsi_its_cam_msgs::VehicleLengthConfidenceIndication VehicleLengthConfidenceIndication_out;
		VehicleLengthConfidenceIndication_out.value = _VehicleLengthConfidenceIndication_in;
		return VehicleLengthConfidenceIndication_out;
	}
}