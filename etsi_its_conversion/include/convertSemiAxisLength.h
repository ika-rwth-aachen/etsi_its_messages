#pragma once

#include <SemiAxisLength.h>
#include <etsi_its_cam_msgs/SemiAxisLength.h>
#include <convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::SemiAxisLength convert_SemiAxisLengthtoRos(const SemiAxisLength_t& _SemiAxisLength_in)
	{
		etsi_its_cam_msgs::SemiAxisLength SemiAxisLength_out;
		convert_toRos(_SemiAxisLength_in, SemiAxisLength_out.value);
		return SemiAxisLength_out;
	}
}