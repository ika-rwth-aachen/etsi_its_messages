#pragma once

#include <etsi_its_cam_coding/YawRateValue.h>
#include <etsi_its_cam_msgs/YawRateValue.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	void convert_YawRateValuetoRos(const YawRateValue_t& _YawRateValue_in, etsi_its_cam_msgs::YawRateValue& _YawRateValue_out)
	{
		convert_toRos(_YawRateValue_in, _YawRateValue_out.value);
	}
	void convert_YawRateValuetoC(const etsi_its_cam_msgs::YawRateValue& _YawRateValue_in, YawRateValue_t& _YawRateValue_out)
	{
		memset(&_YawRateValue_out, 0, sizeof(YawRateValue_t));
		convert_toC(_YawRateValue_in.value, _YawRateValue_out);
	}
}