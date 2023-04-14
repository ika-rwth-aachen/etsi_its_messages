#pragma once

#include <etsi_its_cam_coding/PtActivation.h>
#include <etsi_its_cam_msgs/PtActivation.h>
#include <etsi_its_cam_conversion/convertPtActivationType.h>
#include <etsi_its_cam_conversion/convertPtActivationData.h>

namespace etsi_its_cam_conversion
{
	void convert_PtActivationtoRos(const PtActivation_t& _PtActivation_in, etsi_its_cam_msgs::PtActivation& _PtActivation_out)
	{
		convert_PtActivationTypetoRos(_PtActivation_in.ptActivationType, _PtActivation_out.ptActivationType);
		convert_PtActivationDatatoRos(_PtActivation_in.ptActivationData, _PtActivation_out.ptActivationData);
	}
	void convert_PtActivationtoC(const etsi_its_cam_msgs::PtActivation& _PtActivation_in, PtActivation_t& _PtActivation_out)
	{
		memset(&_PtActivation_out, 0, sizeof(PtActivation_t));
		convert_PtActivationTypetoC(_PtActivation_in.ptActivationType, _PtActivation_out.ptActivationType);
		convert_PtActivationDatatoC(_PtActivation_in.ptActivationData, _PtActivation_out.ptActivationData);
	}
}