#pragma once

#include <CauseCode.h>
#include <etsi_its_cam_msgs/CauseCode.h>
#include <convertCauseCodeType.h>
#include <convertSubCauseCodeType.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::CauseCode convert_CauseCodetoRos(const CauseCode_t& _CauseCode_in)
	{
		etsi_its_cam_msgs::CauseCode CauseCode_out;
		CauseCode_out.causeCode = convert_CauseCodeTypetoRos(_CauseCode_in.causeCode);
		CauseCode_out.subCauseCode = convert_SubCauseCodeTypetoRos(_CauseCode_in.subCauseCode);
		return CauseCode_out;
	}
}