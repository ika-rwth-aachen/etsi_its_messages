#pragma once

#include <etsi_its_cam_coding/Latitude.h>
#include <etsi_its_cam_msgs/Latitude.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::Latitude convert_LatitudetoRos(const Latitude_t& _Latitude_in)
	{
		etsi_its_cam_msgs::Latitude Latitude_out;
		convert_toRos(_Latitude_in, Latitude_out.value);
		return Latitude_out;
	}
	Latitude_t convert_LatitudetoC(const etsi_its_cam_msgs::Latitude& _Latitude_in)
	{
		Latitude_t Latitude_out;
		convert_toC(_Latitude_in.value, Latitude_out);
		return Latitude_out;
	}
}