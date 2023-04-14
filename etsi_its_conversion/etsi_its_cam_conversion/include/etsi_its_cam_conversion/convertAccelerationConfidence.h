#pragma once

#include <etsi_its_cam_coding/AccelerationConfidence.h>
#include <etsi_its_cam_msgs/AccelerationConfidence.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	void convert_AccelerationConfidencetoRos(const AccelerationConfidence_t& _AccelerationConfidence_in, etsi_its_cam_msgs::AccelerationConfidence& _AccelerationConfidence_out)
	{
		convert_toRos(_AccelerationConfidence_in, _AccelerationConfidence_out.value);
	}
	void convert_AccelerationConfidencetoC(const etsi_its_cam_msgs::AccelerationConfidence& _AccelerationConfidence_in, AccelerationConfidence_t& _AccelerationConfidence_out)
	{
		memset(&_AccelerationConfidence_out, 0, sizeof(AccelerationConfidence_t));
		convert_toC(_AccelerationConfidence_in.value, _AccelerationConfidence_out);
	}
}