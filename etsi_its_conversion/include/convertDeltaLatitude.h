#pragma once

#include <DeltaLatitude.h>
#include <etsi_its_cam_msgs/DeltaLatitude.h>
#include <convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::DeltaLatitude convert_DeltaLatitudetoRos(const DeltaLatitude_t& _DeltaLatitude_in)
	{
		etsi_its_cam_msgs::DeltaLatitude DeltaLatitude_out;
		convert_toRos(_DeltaLatitude_in, DeltaLatitude_out.value);
		return DeltaLatitude_out;
	}
}