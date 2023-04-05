#pragma once

#include <etsi_its_cam_coding/SpecialTransportType.h>
#include <etsi_its_cam_msgs/SpecialTransportType.h>
#include <etsi_its_cam_conversion/primitives/convertBIT_STRING.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::SpecialTransportType convert_SpecialTransportTypetoRos(const SpecialTransportType_t& _SpecialTransportType_in)
	{
		etsi_its_cam_msgs::SpecialTransportType SpecialTransportType_out;
		convert_BIT_STRINGtoRos(_SpecialTransportType_in, SpecialTransportType_out.value);
		return SpecialTransportType_out;
	}
	SpecialTransportType_t convert_SpecialTransportTypetoC(const etsi_its_cam_msgs::SpecialTransportType& _SpecialTransportType_in)
	{
		SpecialTransportType_t SpecialTransportType_out;
		convert_BIT_STRINGtoC(_SpecialTransportType_in.value, SpecialTransportType_out);
		return SpecialTransportType_out;
	}
}