#pragma once

#include <etsi_its_cam_coding/YawRateConfidence.h>
#include <etsi_its_cam_msgs/YawRateConfidence.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::YawRateConfidence convert_YawRateConfidencetoRos(const YawRateConfidence_t& _YawRateConfidence_in)
	{
		etsi_its_cam_msgs::YawRateConfidence YawRateConfidence_out;
		YawRateConfidence_out.value = _YawRateConfidence_in;
		return YawRateConfidence_out;
	}
	YawRateConfidence_t convert_YawRateConfidencetoC(const etsi_its_cam_msgs::YawRateConfidence& _YawRateConfidence_in)
	{
		YawRateConfidence_t YawRateConfidence_out;
		memset(&YawRateConfidence_out, 0, sizeof(YawRateConfidence_t));
		YawRateConfidence_out = _YawRateConfidence_in.value;
		return YawRateConfidence_out;
	}
}