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
}