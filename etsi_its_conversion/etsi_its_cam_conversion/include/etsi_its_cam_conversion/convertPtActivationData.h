#pragma once

#include <etsi_its_cam_coding/PtActivationData.h>
#include <etsi_its_cam_msgs/PtActivationData.h>
#include <etsi_its_cam_conversion/primitives/convertOCTET_STRING.h>

namespace etsi_its_cam_conversion
{
	void convert_PtActivationDatatoRos(const PtActivationData_t& _PtActivationData_in, etsi_its_cam_msgs::PtActivationData& _PtActivationData_out)
	{
		convert_OCTET_STRINGtoRos(_PtActivationData_in, _PtActivationData_out.value);
	}
	void convert_PtActivationDatatoC(const etsi_its_cam_msgs::PtActivationData& _PtActivationData_in, PtActivationData_t& _PtActivationData_out)
	{
		memset(&_PtActivationData_out, 0, sizeof(PtActivationData_t));
		convert_OCTET_STRINGtoC(_PtActivationData_in.value, _PtActivationData_out);
	}
}