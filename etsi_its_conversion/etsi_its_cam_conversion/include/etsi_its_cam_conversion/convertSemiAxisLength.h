#pragma once

#include <etsi_its_cam_coding/SemiAxisLength.h>
#include <etsi_its_cam_msgs/SemiAxisLength.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::SemiAxisLength convert_SemiAxisLengthtoRos(const SemiAxisLength_t& _SemiAxisLength_in)
	{
		etsi_its_cam_msgs::SemiAxisLength SemiAxisLength_out;
		convert_toRos(_SemiAxisLength_in, SemiAxisLength_out.value);
		return SemiAxisLength_out;
	}
	SemiAxisLength_t convert_SemiAxisLengthtoC(const etsi_its_cam_msgs::SemiAxisLength& _SemiAxisLength_in)
	{
		SemiAxisLength_t SemiAxisLength_out;
		convert_toC(_SemiAxisLength_in.value, SemiAxisLength_out);
		return SemiAxisLength_out;
	}
}