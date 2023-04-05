#pragma once

#include <etsi_its_cam_coding/YawRateValue.h>
#include <etsi_its_cam_msgs/YawRateValue.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::YawRateValue convert_YawRateValuetoRos(const YawRateValue_t& _YawRateValue_in)
	{
		etsi_its_cam_msgs::YawRateValue YawRateValue_out;
		convert_toRos(_YawRateValue_in, YawRateValue_out.value);
		return YawRateValue_out;
	}
	YawRateValue_t convert_YawRateValuetoC(const etsi_its_cam_msgs::YawRateValue& _YawRateValue_in)
	{
		YawRateValue_t YawRateValue_out;
		convert_toC(_YawRateValue_in.value, YawRateValue_out);
		return YawRateValue_out;
	}
}