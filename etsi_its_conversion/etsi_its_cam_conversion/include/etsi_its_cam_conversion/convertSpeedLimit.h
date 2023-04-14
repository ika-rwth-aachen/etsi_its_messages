#pragma once

#include <etsi_its_cam_coding/SpeedLimit.h>
#include <etsi_its_cam_msgs/SpeedLimit.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	void convert_SpeedLimittoRos(const SpeedLimit_t& _SpeedLimit_in, etsi_its_cam_msgs::SpeedLimit& _SpeedLimit_out)
	{
		convert_toRos(_SpeedLimit_in, _SpeedLimit_out.value);
	}
	void convert_SpeedLimittoC(const etsi_its_cam_msgs::SpeedLimit& _SpeedLimit_in, SpeedLimit_t& _SpeedLimit_out)
	{
		memset(&_SpeedLimit_out, 0, sizeof(SpeedLimit_t));
		convert_toC(_SpeedLimit_in.value, _SpeedLimit_out);
	}
}