#pragma once

#include <etsi_its_cam_coding/SemiAxisLength.h>
#include <etsi_its_cam_msgs/SemiAxisLength.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	void convert_SemiAxisLengthtoRos(const SemiAxisLength_t& _SemiAxisLength_in, etsi_its_cam_msgs::SemiAxisLength& _SemiAxisLength_out)
	{
		convert_toRos(_SemiAxisLength_in, _SemiAxisLength_out.value);
	}
	void convert_SemiAxisLengthtoC(const etsi_its_cam_msgs::SemiAxisLength& _SemiAxisLength_in, SemiAxisLength_t& _SemiAxisLength_out)
	{
		memset(&_SemiAxisLength_out, 0, sizeof(SemiAxisLength_t));
		convert_toC(_SemiAxisLength_in.value, _SemiAxisLength_out);
	}
}