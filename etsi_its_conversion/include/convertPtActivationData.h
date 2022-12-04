#pragma once

#include <PtActivationData.h>
#include <etsi_its_cam_msgs/PtActivationData.h>
#include <convertOCTET_STRING.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::PtActivationData convert_PtActivationDatatoRos(const PtActivationData_t& _PtActivationData_in)
	{
		etsi_its_cam_msgs::PtActivationData PtActivationData_out;
		convert_OCTET_STRINGtoRos(_PtActivationData_in, PtActivationData_out.value);
		return PtActivationData_out;
	}
}