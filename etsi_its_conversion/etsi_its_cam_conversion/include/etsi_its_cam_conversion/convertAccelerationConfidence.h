#pragma once

#include <etsi_its_cam_coding/AccelerationConfidence.h>
#include <etsi_its_cam_msgs/AccelerationConfidence.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::AccelerationConfidence convert_AccelerationConfidencetoRos(const AccelerationConfidence_t& _AccelerationConfidence_in)
	{
		etsi_its_cam_msgs::AccelerationConfidence AccelerationConfidence_out;
		convert_toRos(_AccelerationConfidence_in, AccelerationConfidence_out.value);
		return AccelerationConfidence_out;
	}
	AccelerationConfidence_t convert_AccelerationConfidencetoC(const etsi_its_cam_msgs::AccelerationConfidence& _AccelerationConfidence_in)
	{
		AccelerationConfidence_t AccelerationConfidence_out;
		memset(&AccelerationConfidence_out, 0, sizeof(AccelerationConfidence_t));
		convert_toC(_AccelerationConfidence_in.value, AccelerationConfidence_out);
		return AccelerationConfidence_out;
	}
}