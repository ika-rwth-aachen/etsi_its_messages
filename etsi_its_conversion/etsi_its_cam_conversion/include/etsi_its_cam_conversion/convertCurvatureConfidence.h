#pragma once

#include <etsi_its_cam_coding/CurvatureConfidence.h>
#include <etsi_its_cam_msgs/CurvatureConfidence.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::CurvatureConfidence convert_CurvatureConfidencetoRos(const CurvatureConfidence_t& _CurvatureConfidence_in)
	{
		etsi_its_cam_msgs::CurvatureConfidence CurvatureConfidence_out;
		CurvatureConfidence_out.value = _CurvatureConfidence_in;
		return CurvatureConfidence_out;
	}
}