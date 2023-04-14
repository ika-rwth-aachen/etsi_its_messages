#pragma once

#include <etsi_its_cam_coding/AltitudeConfidence.h>
#include <etsi_its_cam_msgs/AltitudeConfidence.h>

namespace etsi_its_cam_conversion
{
	void convert_AltitudeConfidencetoRos(const AltitudeConfidence_t& _AltitudeConfidence_in, etsi_its_cam_msgs::AltitudeConfidence& _AltitudeConfidence_out)
	{
		_AltitudeConfidence_out.value = _AltitudeConfidence_in;
	}
	void convert_AltitudeConfidencetoC(const etsi_its_cam_msgs::AltitudeConfidence& _AltitudeConfidence_in, AltitudeConfidence_t& _AltitudeConfidence_out)
	{
		memset(&_AltitudeConfidence_out, 0, sizeof(AltitudeConfidence_t));
		_AltitudeConfidence_out = _AltitudeConfidence_in.value;
	}
}