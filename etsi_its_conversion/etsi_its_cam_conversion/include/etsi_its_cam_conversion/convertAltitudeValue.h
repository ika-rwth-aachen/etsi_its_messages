#pragma once

#include <etsi_its_cam_coding/AltitudeValue.h>
#include <etsi_its_cam_msgs/AltitudeValue.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::AltitudeValue convert_AltitudeValuetoRos(const AltitudeValue_t& _AltitudeValue_in)
	{
		etsi_its_cam_msgs::AltitudeValue AltitudeValue_out;
		convert_toRos(_AltitudeValue_in, AltitudeValue_out.value);
		return AltitudeValue_out;
	}
}