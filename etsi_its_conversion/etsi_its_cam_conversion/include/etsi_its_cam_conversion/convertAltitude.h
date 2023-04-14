#pragma once

#include <etsi_its_cam_coding/Altitude.h>
#include <etsi_its_cam_msgs/Altitude.h>
#include <etsi_its_cam_conversion/convertAltitudeValue.h>
#include <etsi_its_cam_conversion/convertAltitudeConfidence.h>

namespace etsi_its_cam_conversion
{
	void convert_AltitudetoRos(const Altitude_t& _Altitude_in, etsi_its_cam_msgs::Altitude& _Altitude_out)
	{
		convert_AltitudeValuetoRos(_Altitude_in.altitudeValue, _Altitude_out.altitudeValue);
		convert_AltitudeConfidencetoRos(_Altitude_in.altitudeConfidence, _Altitude_out.altitudeConfidence);
	}
	void convert_AltitudetoC(const etsi_its_cam_msgs::Altitude& _Altitude_in, Altitude_t& _Altitude_out)
	{
		memset(&_Altitude_out, 0, sizeof(Altitude_t));
		convert_AltitudeValuetoC(_Altitude_in.altitudeValue, _Altitude_out.altitudeValue);
		convert_AltitudeConfidencetoC(_Altitude_in.altitudeConfidence, _Altitude_out.altitudeConfidence);
	}
}