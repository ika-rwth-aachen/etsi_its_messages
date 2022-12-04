#pragma once

#include <PtActivation.h>
#include <etsi_its_cam_msgs/PtActivation.h>
#include <convertPtActivationType.h>
#include <convertPtActivationData.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::PtActivation convert_PtActivationtoRos(const PtActivation_t& _PtActivation_in)
	{
		etsi_its_cam_msgs::PtActivation PtActivation_out;
		PtActivation_out.ptActivationType = convert_PtActivationTypetoRos(_PtActivation_in.ptActivationType);
		PtActivation_out.ptActivationData = convert_PtActivationDatatoRos(_PtActivation_in.ptActivationData);
		return PtActivation_out;
	}
}