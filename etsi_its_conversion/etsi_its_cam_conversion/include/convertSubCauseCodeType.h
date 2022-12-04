#pragma once

#include <SubCauseCodeType.h>
#include <etsi_its_cam_msgs/SubCauseCodeType.h>
#include <primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::SubCauseCodeType convert_SubCauseCodeTypetoRos(const SubCauseCodeType_t& _SubCauseCodeType_in)
	{
		etsi_its_cam_msgs::SubCauseCodeType SubCauseCodeType_out;
		convert_toRos(_SubCauseCodeType_in, SubCauseCodeType_out.value);
		return SubCauseCodeType_out;
	}
}