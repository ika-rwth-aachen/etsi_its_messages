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
	AltitudeValue_t convert_AltitudeValuetoC(const etsi_its_cam_msgs::AltitudeValue& _AltitudeValue_in)
	{
		AltitudeValue_t AltitudeValue_out;
		memset(&AltitudeValue_out, 0, sizeof(AltitudeValue_t));
		convert_toC(_AltitudeValue_in.value, AltitudeValue_out);
		return AltitudeValue_out;
	}
}