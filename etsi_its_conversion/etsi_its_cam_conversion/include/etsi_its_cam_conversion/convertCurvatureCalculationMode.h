#pragma once

#include <etsi_its_cam_coding/CurvatureCalculationMode.h>
#include <etsi_its_cam_msgs/CurvatureCalculationMode.h>

namespace etsi_its_cam_conversion
{
	void convert_CurvatureCalculationModetoRos(const CurvatureCalculationMode_t& _CurvatureCalculationMode_in, etsi_its_cam_msgs::CurvatureCalculationMode& _CurvatureCalculationMode_out)
	{
		_CurvatureCalculationMode_out.value = _CurvatureCalculationMode_in;
	}
	void convert_CurvatureCalculationModetoC(const etsi_its_cam_msgs::CurvatureCalculationMode& _CurvatureCalculationMode_in, CurvatureCalculationMode_t& _CurvatureCalculationMode_out)
	{
		memset(&_CurvatureCalculationMode_out, 0, sizeof(CurvatureCalculationMode_t));
		_CurvatureCalculationMode_out = _CurvatureCalculationMode_in.value;
	}
}