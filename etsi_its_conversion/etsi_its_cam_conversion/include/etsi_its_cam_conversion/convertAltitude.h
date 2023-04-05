#pragma once

#include <etsi_its_cam_coding/Altitude.h>
#include <etsi_its_cam_msgs/Altitude.h>
#include <etsi_its_cam_conversion/convertAltitudeValue.h>
#include <etsi_its_cam_conversion/convertAltitudeConfidence.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::Altitude convert_AltitudetoRos(const Altitude_t& _Altitude_in)
	{
		etsi_its_cam_msgs::Altitude Altitude_out;
		Altitude_out.altitudeValue = convert_AltitudeValuetoRos(_Altitude_in.altitudeValue);
		Altitude_out.altitudeConfidence = convert_AltitudeConfidencetoRos(_Altitude_in.altitudeConfidence);
		return Altitude_out;
	}
	Altitude_t convert_AltitudetoC(const etsi_its_cam_msgs::Altitude& _Altitude_in)
	{
		Altitude_t Altitude_out;
		memset(&Altitude_out, 0, sizeof(Altitude_t));
		Altitude_out.altitudeValue = convert_AltitudeValuetoC(_Altitude_in.altitudeValue);
		Altitude_out.altitudeConfidence = convert_AltitudeConfidencetoC(_Altitude_in.altitudeConfidence);
		return Altitude_out;
	}
}