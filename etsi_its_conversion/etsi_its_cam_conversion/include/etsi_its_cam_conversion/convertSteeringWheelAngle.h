#pragma once

#include <etsi_its_cam_coding/SteeringWheelAngle.h>
#include <etsi_its_cam_msgs/SteeringWheelAngle.h>
#include <etsi_its_cam_conversion/convertSteeringWheelAngleValue.h>
#include <etsi_its_cam_conversion/convertSteeringWheelAngleConfidence.h>

namespace etsi_its_cam_conversion
{
	void convert_SteeringWheelAngletoRos(const SteeringWheelAngle_t& _SteeringWheelAngle_in, etsi_its_cam_msgs::SteeringWheelAngle& _SteeringWheelAngle_out)
	{
		convert_SteeringWheelAngleValuetoRos(_SteeringWheelAngle_in.steeringWheelAngleValue, _SteeringWheelAngle_out.steeringWheelAngleValue);
		convert_SteeringWheelAngleConfidencetoRos(_SteeringWheelAngle_in.steeringWheelAngleConfidence, _SteeringWheelAngle_out.steeringWheelAngleConfidence);
	}
	void convert_SteeringWheelAngletoC(const etsi_its_cam_msgs::SteeringWheelAngle& _SteeringWheelAngle_in, SteeringWheelAngle_t& _SteeringWheelAngle_out)
	{
		memset(&_SteeringWheelAngle_out, 0, sizeof(SteeringWheelAngle_t));
		convert_SteeringWheelAngleValuetoC(_SteeringWheelAngle_in.steeringWheelAngleValue, _SteeringWheelAngle_out.steeringWheelAngleValue);
		convert_SteeringWheelAngleConfidencetoC(_SteeringWheelAngle_in.steeringWheelAngleConfidence, _SteeringWheelAngle_out.steeringWheelAngleConfidence);
	}
}