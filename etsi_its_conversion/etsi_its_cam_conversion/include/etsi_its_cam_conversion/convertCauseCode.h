#pragma once

#include <etsi_its_cam_coding/CauseCode.h>
#include <etsi_its_cam_msgs/CauseCode.h>
#include <etsi_its_cam_conversion/convertCauseCodeType.h>
#include <etsi_its_cam_conversion/convertSubCauseCodeType.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::CauseCode convert_CauseCodetoRos(const CauseCode_t& _CauseCode_in)
	{
		etsi_its_cam_msgs::CauseCode CauseCode_out;
		CauseCode_out.causeCode = convert_CauseCodeTypetoRos(_CauseCode_in.causeCode);
		CauseCode_out.subCauseCode = convert_SubCauseCodeTypetoRos(_CauseCode_in.subCauseCode);
		return CauseCode_out;
	}
	CauseCode_t convert_CauseCodetoC(const etsi_its_cam_msgs::CauseCode& _CauseCode_in)
	{
		CauseCode_t CauseCode_out;
		memset(&CauseCode_out, 0, sizeof(CauseCode_t));
		CauseCode_out.causeCode = convert_CauseCodeTypetoC(_CauseCode_in.causeCode);
		CauseCode_out.subCauseCode = convert_SubCauseCodeTypetoC(_CauseCode_in.subCauseCode);
		return CauseCode_out;
	}
}