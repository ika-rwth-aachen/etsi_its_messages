#pragma once

#include <etsi_its_cam_coding/CauseCode.h>
#include <etsi_its_cam_msgs/CauseCode.h>
#include <etsi_its_cam_conversion/convertCauseCodeType.h>
#include <etsi_its_cam_conversion/convertSubCauseCodeType.h>

namespace etsi_its_cam_conversion
{
	void convert_CauseCodetoRos(const CauseCode_t& _CauseCode_in, etsi_its_cam_msgs::CauseCode& _CauseCode_out)
	{
		convert_CauseCodeTypetoRos(_CauseCode_in.causeCode, _CauseCode_out.causeCode);
		convert_SubCauseCodeTypetoRos(_CauseCode_in.subCauseCode, _CauseCode_out.subCauseCode);
	}
	void convert_CauseCodetoC(const etsi_its_cam_msgs::CauseCode& _CauseCode_in, CauseCode_t& _CauseCode_out)
	{
		memset(&_CauseCode_out, 0, sizeof(CauseCode_t));
		convert_CauseCodeTypetoC(_CauseCode_in.causeCode, _CauseCode_out.causeCode);
		convert_SubCauseCodeTypetoC(_CauseCode_in.subCauseCode, _CauseCode_out.subCauseCode);
	}
}