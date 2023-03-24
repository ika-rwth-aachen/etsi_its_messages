#pragma once

#include <etsi_its_cam_coding/VehicleLengthValue.h>
#include <etsi_its_cam_msgs/VehicleLengthValue.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::VehicleLengthValue convert_VehicleLengthValuetoRos(const VehicleLengthValue_t& _VehicleLengthValue_in)
	{
		etsi_its_cam_msgs::VehicleLengthValue VehicleLengthValue_out;
		convert_toRos(_VehicleLengthValue_in, VehicleLengthValue_out.value);
		return VehicleLengthValue_out;
	}
}