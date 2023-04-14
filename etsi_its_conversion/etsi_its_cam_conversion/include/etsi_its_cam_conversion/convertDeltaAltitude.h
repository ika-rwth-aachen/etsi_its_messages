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
	DeltaAltitude_t convert_DeltaAltitudetoC(const etsi_its_cam_msgs::DeltaAltitude& _DeltaAltitude_in)
	{
		DeltaAltitude_t DeltaAltitude_out;
		memset(&DeltaAltitude_out, 0, sizeof(DeltaAltitude_t));
		convert_toC(_DeltaAltitude_in.value, DeltaAltitude_out);
		return DeltaAltitude_out;
	}
}