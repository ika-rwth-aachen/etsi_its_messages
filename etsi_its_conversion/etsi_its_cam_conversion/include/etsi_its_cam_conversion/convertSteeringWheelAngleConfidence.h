#pragma once

#include <etsi_its_cam_coding/SteeringWheelAngleConfidence.h>
#include <etsi_its_cam_msgs/SteeringWheelAngleConfidence.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::SteeringWheelAngleConfidence convert_SteeringWheelAngleConfidencetoRos(const SteeringWheelAngleConfidence_t& _SteeringWheelAngleConfidence_in)
	{
		etsi_its_cam_msgs::SteeringWheelAngleConfidence SteeringWheelAngleConfidence_out;
		convert_toRos(_SteeringWheelAngleConfidence_in, SteeringWheelAngleConfidence_out.value);
		return SteeringWheelAngleConfidence_out;
	}
	SteeringWheelAngleConfidence_t convert_SteeringWheelAngleConfidencetoC(const etsi_its_cam_msgs::SteeringWheelAngleConfidence& _SteeringWheelAngleConfidence_in)
	{
		SteeringWheelAngleConfidence_t SteeringWheelAngleConfidence_out;
		convert_toC(_SteeringWheelAngleConfidence_in.value, SteeringWheelAngleConfidence_out);
		return SteeringWheelAngleConfidence_out;
	}
}