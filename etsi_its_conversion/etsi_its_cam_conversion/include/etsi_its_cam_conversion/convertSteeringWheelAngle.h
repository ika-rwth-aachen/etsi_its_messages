#pragma once

#include <etsi_its_cam_coding/SteeringWheelAngle.h>
#include <etsi_its_cam_msgs/SteeringWheelAngle.h>
#include <etsi_its_cam_conversion/convertSteeringWheelAngleValue.h>
#include <etsi_its_cam_conversion/convertSteeringWheelAngleConfidence.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::SteeringWheelAngle convert_SteeringWheelAngletoRos(const SteeringWheelAngle_t& _SteeringWheelAngle_in)
	{
		etsi_its_cam_msgs::SteeringWheelAngle SteeringWheelAngle_out;
		SteeringWheelAngle_out.steeringWheelAngleValue = convert_SteeringWheelAngleValuetoRos(_SteeringWheelAngle_in.steeringWheelAngleValue);
		SteeringWheelAngle_out.steeringWheelAngleConfidence = convert_SteeringWheelAngleConfidencetoRos(_SteeringWheelAngle_in.steeringWheelAngleConfidence);
		return SteeringWheelAngle_out;
	}
}