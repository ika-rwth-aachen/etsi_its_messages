#pragma once

#include <etsi_its_cam_coding/PtActivationData.h>
#include <etsi_its_cam_msgs/PtActivationData.h>
#include <etsi_its_cam_conversion/primitives/convertOCTET_STRING.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::PtActivationData convert_PtActivationDatatoRos(const PtActivationData_t& _PtActivationData_in)
	{
		etsi_its_cam_msgs::PtActivationData PtActivationData_out;
		convert_OCTET_STRINGtoRos(_PtActivationData_in, PtActivationData_out.value);
		return PtActivationData_out;
	}
	PtActivationData_t convert_PtActivationDatatoC(const etsi_its_cam_msgs::PtActivationData& _PtActivationData_in)
	{
		PtActivationData_t PtActivationData_out;
		memset(&PtActivationData_out, 0, sizeof(PtActivationData_t));
		convert_OCTET_STRINGtoC(_PtActivationData_in.value, PtActivationData_out);
		return PtActivationData_out;
	}
}