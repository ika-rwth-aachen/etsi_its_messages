#pragma once

#include <BasicVehicleContainerHighFrequency.h>
#include <etsi_its_cam_msgs/BasicVehicleContainerHighFrequency.h>
#include <convertHeading.h>
#include <convertSpeed.h>
#include <convertDriveDirection.h>
#include <convertVehicleLength.h>
#include <convertVehicleWidth.h>
#include <convertLongitudinalAcceleration.h>
#include <convertCurvature.h>
#include <convertCurvatureCalculationMode.h>
#include <convertYawRate.h>
#include <convertAccelerationControl.h>
#include <convertLanePosition.h>
#include <convertSteeringWheelAngle.h>
#include <convertLateralAcceleration.h>
#include <convertVerticalAcceleration.h>
#include <convertPerformanceClass.h>
#include <convertCenDsrcTollingZone.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::BasicVehicleContainerHighFrequency convert_BasicVehicleContainerHighFrequencytoRos(const BasicVehicleContainerHighFrequency_t& _BasicVehicleContainerHighFrequency_in)
	{
		etsi_its_cam_msgs::BasicVehicleContainerHighFrequency BasicVehicleContainerHighFrequency_out;
		BasicVehicleContainerHighFrequency_out.heading = convert_HeadingtoRos(_BasicVehicleContainerHighFrequency_in.heading);
		BasicVehicleContainerHighFrequency_out.speed = convert_SpeedtoRos(_BasicVehicleContainerHighFrequency_in.speed);
		BasicVehicleContainerHighFrequency_out.driveDirection = convert_DriveDirectiontoRos(_BasicVehicleContainerHighFrequency_in.driveDirection);
		BasicVehicleContainerHighFrequency_out.vehicleLength = convert_VehicleLengthtoRos(_BasicVehicleContainerHighFrequency_in.vehicleLength);
		BasicVehicleContainerHighFrequency_out.vehicleWidth = convert_VehicleWidthtoRos(_BasicVehicleContainerHighFrequency_in.vehicleWidth);
		BasicVehicleContainerHighFrequency_out.longitudinalAcceleration = convert_LongitudinalAccelerationtoRos(_BasicVehicleContainerHighFrequency_in.longitudinalAcceleration);
		BasicVehicleContainerHighFrequency_out.curvature = convert_CurvaturetoRos(_BasicVehicleContainerHighFrequency_in.curvature);
		BasicVehicleContainerHighFrequency_out.curvatureCalculationMode = convert_CurvatureCalculationModetoRos(_BasicVehicleContainerHighFrequency_in.curvatureCalculationMode);
		BasicVehicleContainerHighFrequency_out.yawRate = convert_YawRatetoRos(_BasicVehicleContainerHighFrequency_in.yawRate);
		if(_BasicVehicleContainerHighFrequency_in.accelerationControl)
		{
			BasicVehicleContainerHighFrequency_out.accelerationControl = convert_AccelerationControltoRos(*_BasicVehicleContainerHighFrequency_in.accelerationControl);
		}
		if(_BasicVehicleContainerHighFrequency_in.lanePosition)
		{
			BasicVehicleContainerHighFrequency_out.lanePosition = convert_LanePositiontoRos(*_BasicVehicleContainerHighFrequency_in.lanePosition);
		}
		if(_BasicVehicleContainerHighFrequency_in.steeringWheelAngle)
		{
			BasicVehicleContainerHighFrequency_out.steeringWheelAngle = convert_SteeringWheelAngletoRos(*_BasicVehicleContainerHighFrequency_in.steeringWheelAngle);
		}
		if(_BasicVehicleContainerHighFrequency_in.lateralAcceleration)
		{
			BasicVehicleContainerHighFrequency_out.lateralAcceleration = convert_LateralAccelerationtoRos(*_BasicVehicleContainerHighFrequency_in.lateralAcceleration);
		}
		if(_BasicVehicleContainerHighFrequency_in.verticalAcceleration)
		{
			BasicVehicleContainerHighFrequency_out.verticalAcceleration = convert_VerticalAccelerationtoRos(*_BasicVehicleContainerHighFrequency_in.verticalAcceleration);
		}
		if(_BasicVehicleContainerHighFrequency_in.performanceClass)
		{
			BasicVehicleContainerHighFrequency_out.performanceClass = convert_PerformanceClasstoRos(*_BasicVehicleContainerHighFrequency_in.performanceClass);
		}
		if(_BasicVehicleContainerHighFrequency_in.cenDsrcTollingZone)
		{
			BasicVehicleContainerHighFrequency_out.cenDsrcTollingZone = convert_CenDsrcTollingZonetoRos(*_BasicVehicleContainerHighFrequency_in.cenDsrcTollingZone);
		}
		return BasicVehicleContainerHighFrequency_out;
	}
}