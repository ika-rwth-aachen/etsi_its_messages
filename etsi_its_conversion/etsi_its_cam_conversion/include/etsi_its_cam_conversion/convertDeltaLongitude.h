#pragma once

#include <etsi_its_cam_coding/DeltaLongitude.h>
#include <etsi_its_cam_msgs/DeltaLongitude.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::DeltaLongitude convert_DeltaLongitudetoRos(const DeltaLongitude_t& _DeltaLongitude_in)
	{
		etsi_its_cam_msgs::DeltaLongitude DeltaLongitude_out;
		convert_toRos(_DeltaLongitude_in, DeltaLongitude_out.value);
		return DeltaLongitude_out;
	}
	DeltaLongitude_t convert_DeltaLongitudetoC(const etsi_its_cam_msgs::DeltaLongitude& _DeltaLongitude_in)
	{
		DeltaLongitude_t DeltaLongitude_out;
		convert_toC(_DeltaLongitude_in.value, DeltaLongitude_out);
		return DeltaLongitude_out;
	}
}