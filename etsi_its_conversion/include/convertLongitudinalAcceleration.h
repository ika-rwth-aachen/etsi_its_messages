#pragma once

#include <LongitudinalAcceleration.h>
#include <etsi_its_cam_msgs/LongitudinalAcceleration.h>
#include <convertLongitudinalAccelerationValue.h>
#include <convertAccelerationConfidence.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::LongitudinalAcceleration convert_LongitudinalAccelerationtoRos(const LongitudinalAcceleration_t& _LongitudinalAcceleration_in)
	{
		etsi_its_cam_msgs::LongitudinalAcceleration LongitudinalAcceleration_out;
		LongitudinalAcceleration_out.longitudinalAccelerationValue = convert_LongitudinalAccelerationValuetoRos(_LongitudinalAcceleration_in.longitudinalAccelerationValue);
		LongitudinalAcceleration_out.longitudinalAccelerationConfidence = convert_AccelerationConfidencetoRos(_LongitudinalAcceleration_in.longitudinalAccelerationConfidence);
		return LongitudinalAcceleration_out;
	}
}