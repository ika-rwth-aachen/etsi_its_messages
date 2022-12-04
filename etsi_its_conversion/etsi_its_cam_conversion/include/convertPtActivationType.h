#pragma once

#include <PtActivationType.h>
#include <etsi_its_cam_msgs/PtActivationType.h>
#include <primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::PtActivationType convert_PtActivationTypetoRos(const PtActivationType_t& _PtActivationType_in)
	{
		etsi_its_cam_msgs::PtActivationType PtActivationType_out;
		convert_toRos(_PtActivationType_in, PtActivationType_out.value);
		return PtActivationType_out;
	}
}