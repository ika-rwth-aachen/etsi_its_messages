#pragma once

#include <etsi_its_cam_coding/SpeedConfidence.h>
#include <etsi_its_cam_msgs/SpeedConfidence.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::SpeedConfidence convert_SpeedConfidencetoRos(const SpeedConfidence_t& _SpeedConfidence_in)
	{
		etsi_its_cam_msgs::SpeedConfidence SpeedConfidence_out;
		convert_toRos(_SpeedConfidence_in, SpeedConfidence_out.value);
		return SpeedConfidence_out;
	}
	SpeedConfidence_t convert_SpeedConfidencetoC(const etsi_its_cam_msgs::SpeedConfidence& _SpeedConfidence_in)
	{
		SpeedConfidence_t SpeedConfidence_out;
		convert_toC(_SpeedConfidence_in.value, SpeedConfidence_out);
		return SpeedConfidence_out;
	}
}