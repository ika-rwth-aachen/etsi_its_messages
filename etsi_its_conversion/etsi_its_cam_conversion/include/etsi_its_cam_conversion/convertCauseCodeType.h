#pragma once

#include <etsi_its_cam_coding/CauseCodeType.h>
#include <etsi_its_cam_msgs/CauseCodeType.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::CauseCodeType convert_CauseCodeTypetoRos(const CauseCodeType_t& _CauseCodeType_in)
	{
		etsi_its_cam_msgs::CauseCodeType CauseCodeType_out;
		convert_toRos(_CauseCodeType_in, CauseCodeType_out.value);
		return CauseCodeType_out;
	}
	CauseCodeType_t convert_CauseCodeTypetoC(const etsi_its_cam_msgs::CauseCodeType& _CauseCodeType_in)
	{
		CauseCodeType_t CauseCodeType_out;
		memset(&CauseCodeType_out, 0, sizeof(CauseCodeType_t));
		convert_toC(_CauseCodeType_in.value, CauseCodeType_out);
		return CauseCodeType_out;
	}
}