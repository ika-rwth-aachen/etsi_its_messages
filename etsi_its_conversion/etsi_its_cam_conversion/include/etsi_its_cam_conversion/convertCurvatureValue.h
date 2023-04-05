#pragma once

#include <etsi_its_cam_coding/CurvatureValue.h>
#include <etsi_its_cam_msgs/CurvatureValue.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::CurvatureValue convert_CurvatureValuetoRos(const CurvatureValue_t& _CurvatureValue_in)
	{
		etsi_its_cam_msgs::CurvatureValue CurvatureValue_out;
		convert_toRos(_CurvatureValue_in, CurvatureValue_out.value);
		return CurvatureValue_out;
	}
	CurvatureValue_t convert_CurvatureValuetoC(const etsi_its_cam_msgs::CurvatureValue& _CurvatureValue_in)
	{
		CurvatureValue_t CurvatureValue_out;
		memset(&CurvatureValue_out, 0, sizeof(CurvatureValue_t));
		convert_toC(_CurvatureValue_in.value, CurvatureValue_out);
		return CurvatureValue_out;
	}
}