#pragma once

#include <etsi_its_cam_coding/CurvatureValue.h>
#include <etsi_its_cam_msgs/CurvatureValue.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	void convert_CurvatureValuetoRos(const CurvatureValue_t& _CurvatureValue_in, etsi_its_cam_msgs::CurvatureValue& _CurvatureValue_out)
	{
		convert_toRos(_CurvatureValue_in, _CurvatureValue_out.value);
	}
	void convert_CurvatureValuetoC(const etsi_its_cam_msgs::CurvatureValue& _CurvatureValue_in, CurvatureValue_t& _CurvatureValue_out)
	{
		memset(&_CurvatureValue_out, 0, sizeof(CurvatureValue_t));
		convert_toC(_CurvatureValue_in.value, _CurvatureValue_out);
	}
}