#pragma once

#include <CauseCodeType.h>
#include <etsi_its_cam_msgs/CauseCodeType.h>
#include <convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::CauseCodeType convert_CauseCodeTypetoRos(const CauseCodeType_t& _CauseCodeType_in)
	{
		etsi_its_cam_msgs::CauseCodeType CauseCodeType_out;
		convert_toRos(_CauseCodeType_in, CauseCodeType_out.value);
		return CauseCodeType_out;
	}
}