#pragma once

#include <etsi_its_cam_coding/Longitude.h>
#include <etsi_its_cam_msgs/Longitude.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::Longitude convert_LongitudetoRos(const Longitude_t& _Longitude_in)
	{
		etsi_its_cam_msgs::Longitude Longitude_out;
		convert_toRos(_Longitude_in, Longitude_out.value);
		return Longitude_out;
	}
}