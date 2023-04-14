#pragma once

#include <etsi_its_cam_coding/SpeedConfidence.h>
#include <etsi_its_cam_msgs/SpeedConfidence.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	void convert_SpeedConfidencetoRos(const SpeedConfidence_t& _SpeedConfidence_in, etsi_its_cam_msgs::SpeedConfidence& _SpeedConfidence_out)
	{
		convert_toRos(_SpeedConfidence_in, _SpeedConfidence_out.value);
	}
	void convert_SpeedConfidencetoC(const etsi_its_cam_msgs::SpeedConfidence& _SpeedConfidence_in, SpeedConfidence_t& _SpeedConfidence_out)
	{
		memset(&_SpeedConfidence_out, 0, sizeof(SpeedConfidence_t));
		convert_toC(_SpeedConfidence_in.value, _SpeedConfidence_out);
	}
}