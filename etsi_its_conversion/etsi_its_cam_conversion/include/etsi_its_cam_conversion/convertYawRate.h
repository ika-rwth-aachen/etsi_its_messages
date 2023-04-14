#pragma once

#include <etsi_its_cam_coding/YawRate.h>
#include <etsi_its_cam_msgs/YawRate.h>
#include <etsi_its_cam_conversion/convertYawRateValue.h>
#include <etsi_its_cam_conversion/convertYawRateConfidence.h>

namespace etsi_its_cam_conversion
{
	void convert_YawRatetoRos(const YawRate_t& _YawRate_in, etsi_its_cam_msgs::YawRate& _YawRate_out)
	{
		convert_YawRateValuetoRos(_YawRate_in.yawRateValue, _YawRate_out.yawRateValue);
		convert_YawRateConfidencetoRos(_YawRate_in.yawRateConfidence, _YawRate_out.yawRateConfidence);
	}
	void convert_YawRatetoC(const etsi_its_cam_msgs::YawRate& _YawRate_in, YawRate_t& _YawRate_out)
	{
		memset(&_YawRate_out, 0, sizeof(YawRate_t));
		convert_YawRateValuetoC(_YawRate_in.yawRateValue, _YawRate_out.yawRateValue);
		convert_YawRateConfidencetoC(_YawRate_in.yawRateConfidence, _YawRate_out.yawRateConfidence);
	}
}