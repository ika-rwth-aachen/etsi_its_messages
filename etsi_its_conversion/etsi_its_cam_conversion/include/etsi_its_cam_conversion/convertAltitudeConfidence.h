#pragma once

#include <etsi_its_cam_coding/AltitudeConfidence.h>
#include <etsi_its_cam_msgs/AltitudeConfidence.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::AltitudeConfidence convert_AltitudeConfidencetoRos(const AltitudeConfidence_t& _AltitudeConfidence_in)
	{
		etsi_its_cam_msgs::AltitudeConfidence AltitudeConfidence_out;
		AltitudeConfidence_out.value = _AltitudeConfidence_in;
		return AltitudeConfidence_out;
	}
}