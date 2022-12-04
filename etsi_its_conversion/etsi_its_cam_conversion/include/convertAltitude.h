#pragma once

#include <Altitude.h>
#include <etsi_its_cam_msgs/Altitude.h>
#include <convertAltitudeValue.h>
#include <convertAltitudeConfidence.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::Altitude convert_AltitudetoRos(const Altitude_t& _Altitude_in)
	{
		etsi_its_cam_msgs::Altitude Altitude_out;
		Altitude_out.altitudeValue = convert_AltitudeValuetoRos(_Altitude_in.altitudeValue);
		Altitude_out.altitudeConfidence = convert_AltitudeConfidencetoRos(_Altitude_in.altitudeConfidence);
		return Altitude_out;
	}
}