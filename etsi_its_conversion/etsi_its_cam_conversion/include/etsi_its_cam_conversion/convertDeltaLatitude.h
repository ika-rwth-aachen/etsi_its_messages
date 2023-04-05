#pragma once

#include <etsi_its_cam_coding/DeltaLatitude.h>
#include <etsi_its_cam_msgs/DeltaLatitude.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::DeltaLatitude convert_DeltaLatitudetoRos(const DeltaLatitude_t& _DeltaLatitude_in)
	{
		etsi_its_cam_msgs::DeltaLatitude DeltaLatitude_out;
		convert_toRos(_DeltaLatitude_in, DeltaLatitude_out.value);
		return DeltaLatitude_out;
	}
	DeltaLatitude_t convert_DeltaLatitudetoC(const etsi_its_cam_msgs::DeltaLatitude& _DeltaLatitude_in)
	{
		DeltaLatitude_t DeltaLatitude_out;
		convert_toC(_DeltaLatitude_in.value, DeltaLatitude_out);
		return DeltaLatitude_out;
	}
}