#pragma once

#include <DeltaAltitude.h>
#include <etsi_its_cam_msgs/DeltaAltitude.h>
#include <primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::DeltaAltitude convert_DeltaAltitudetoRos(const DeltaAltitude_t& _DeltaAltitude_in)
	{
		etsi_its_cam_msgs::DeltaAltitude DeltaAltitude_out;
		convert_toRos(_DeltaAltitude_in, DeltaAltitude_out.value);
		return DeltaAltitude_out;
	}
}