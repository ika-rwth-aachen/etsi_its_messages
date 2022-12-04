#pragma once

#include <AccelerationConfidence.h>
#include <etsi_its_cam_msgs/AccelerationConfidence.h>
#include <convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::AccelerationConfidence convert_AccelerationConfidencetoRos(const AccelerationConfidence_t& _AccelerationConfidence_in)
	{
		etsi_its_cam_msgs::AccelerationConfidence AccelerationConfidence_out;
		convert_toRos(_AccelerationConfidence_in, AccelerationConfidence_out.value);
		return AccelerationConfidence_out;
	}
}