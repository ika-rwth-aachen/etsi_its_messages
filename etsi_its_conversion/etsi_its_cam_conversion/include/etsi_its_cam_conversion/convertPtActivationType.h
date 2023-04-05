#pragma once

#include <etsi_its_cam_coding/PtActivationType.h>
#include <etsi_its_cam_msgs/PtActivationType.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::PtActivationType convert_PtActivationTypetoRos(const PtActivationType_t& _PtActivationType_in)
	{
		etsi_its_cam_msgs::PtActivationType PtActivationType_out;
		convert_toRos(_PtActivationType_in, PtActivationType_out.value);
		return PtActivationType_out;
	}
	PtActivationType_t convert_PtActivationTypetoC(const etsi_its_cam_msgs::PtActivationType& _PtActivationType_in)
	{
		PtActivationType_t PtActivationType_out;
		convert_toC(_PtActivationType_in.value, PtActivationType_out);
		return PtActivationType_out;
	}
}