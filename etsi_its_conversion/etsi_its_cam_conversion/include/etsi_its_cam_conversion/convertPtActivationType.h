#pragma once

#include <etsi_its_cam_coding/PtActivationType.h>
#include <etsi_its_cam_msgs/PtActivationType.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	void convert_PtActivationTypetoRos(const PtActivationType_t& _PtActivationType_in, etsi_its_cam_msgs::PtActivationType& _PtActivationType_out)
	{
		convert_toRos(_PtActivationType_in, _PtActivationType_out.value);
	}
	void convert_PtActivationTypetoC(const etsi_its_cam_msgs::PtActivationType& _PtActivationType_in, PtActivationType_t& _PtActivationType_out)
	{
		memset(&_PtActivationType_out, 0, sizeof(PtActivationType_t));
		convert_toC(_PtActivationType_in.value, _PtActivationType_out);
	}
}