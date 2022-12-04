#pragma once

#include <DeltaLongitude.h>
#include <etsi_its_cam_msgs/DeltaLongitude.h>
#include <convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::DeltaLongitude convert_DeltaLongitudetoRos(const DeltaLongitude_t& _DeltaLongitude_in)
	{
		etsi_its_cam_msgs::DeltaLongitude DeltaLongitude_out;
		convert_toRos(_DeltaLongitude_in, DeltaLongitude_out.value);
		return DeltaLongitude_out;
	}
}