#pragma once

#include <YawRateConfidence.h>
#include <etsi_its_cam_msgs/YawRateConfidence.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::YawRateConfidence convert_YawRateConfidencetoRos(const YawRateConfidence_t& _YawRateConfidence_in)
	{
		etsi_its_cam_msgs::YawRateConfidence YawRateConfidence_out;
		YawRateConfidence_out.value = _YawRateConfidence_in;
		return YawRateConfidence_out;
	}
}