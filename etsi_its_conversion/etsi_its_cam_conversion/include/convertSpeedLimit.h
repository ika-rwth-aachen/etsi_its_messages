#pragma once

#include <SpeedLimit.h>
#include <etsi_its_cam_msgs/SpeedLimit.h>
#include <primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::SpeedLimit convert_SpeedLimittoRos(const SpeedLimit_t& _SpeedLimit_in)
	{
		etsi_its_cam_msgs::SpeedLimit SpeedLimit_out;
		convert_toRos(_SpeedLimit_in, SpeedLimit_out.value);
		return SpeedLimit_out;
	}
}