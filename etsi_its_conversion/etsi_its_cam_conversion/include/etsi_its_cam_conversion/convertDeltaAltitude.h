#pragma once

#include <etsi_its_cam_coding/DeltaAltitude.h>
#include <etsi_its_cam_msgs/DeltaAltitude.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::DeltaAltitude convert_DeltaAltitudetoRos(const DeltaAltitude_t& _DeltaAltitude_in)
	{
		etsi_its_cam_msgs::DeltaAltitude DeltaAltitude_out;
		convert_toRos(_DeltaAltitude_in, DeltaAltitude_out.value);
		return DeltaAltitude_out;
	}
}