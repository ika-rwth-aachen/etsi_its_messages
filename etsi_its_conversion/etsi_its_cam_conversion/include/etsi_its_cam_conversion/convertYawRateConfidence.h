#pragma once

#include <etsi_its_cam_coding/YawRateConfidence.h>
#include <etsi_its_cam_msgs/YawRateConfidence.h>

namespace etsi_its_cam_conversion
{
	void convert_YawRateConfidencetoRos(const YawRateConfidence_t& _YawRateConfidence_in, etsi_its_cam_msgs::YawRateConfidence& _YawRateConfidence_out)
	{
		_YawRateConfidence_out.value = _YawRateConfidence_in;
	}
	void convert_YawRateConfidencetoC(const etsi_its_cam_msgs::YawRateConfidence& _YawRateConfidence_in, YawRateConfidence_t& _YawRateConfidence_out)
	{
		memset(&_YawRateConfidence_out, 0, sizeof(YawRateConfidence_t));
		_YawRateConfidence_out = _YawRateConfidence_in.value;
	}
}