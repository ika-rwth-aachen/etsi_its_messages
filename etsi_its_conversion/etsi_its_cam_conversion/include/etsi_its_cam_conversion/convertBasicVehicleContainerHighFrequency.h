#pragma once

#include <etsi_its_cam_coding/BasicVehicleContainerHighFrequency.h>
#include <etsi_its_cam_msgs/BasicVehicleContainerHighFrequency.h>
#include <etsi_its_cam_conversion/convertHeading.h>
#include <etsi_its_cam_conversion/convertSpeed.h>
#include <etsi_its_cam_conversion/convertDriveDirection.h>
#include <etsi_its_cam_conversion/convertVehicleLength.h>
#include <etsi_its_cam_conversion/convertVehicleWidth.h>
#include <etsi_its_cam_conversion/convertLongitudinalAcceleration.h>
#include <etsi_its_cam_conversion/convertCurvature.h>
#include <etsi_its_cam_conversion/convertCurvatureCalculationMode.h>
#include <etsi_its_cam_conversion/convertYawRate.h>
#include <etsi_its_cam_conversion/convertAccelerationControl.h>
#include <etsi_its_cam_conversion/convertLanePosition.h>
#include <etsi_its_cam_conversion/convertSteeringWheelAngle.h>
#include <etsi_its_cam_conversion/convertLateralAcceleration.h>
#include <etsi_its_cam_conversion/convertVerticalAcceleration.h>
#include <etsi_its_cam_conversion/convertPerformanceClass.h>
#include <etsi_its_cam_conversion/convertCenDsrcTollingZone.h>

namespace etsi_its_cam_conversion
{
	void convert_BasicVehicleContainerHighFrequencytoRos(const BasicVehicleContainerHighFrequency_t& _BasicVehicleContainerHighFrequency_in, etsi_its_cam_msgs::BasicVehicleContainerHighFrequency& _BasicVehicleContainerHighFrequency_out)
	{
		convert_HeadingtoRos(_BasicVehicleContainerHighFrequency_in.heading, _BasicVehicleContainerHighFrequency_out.heading);
		convert_SpeedtoRos(_BasicVehicleContainerHighFrequency_in.speed, _BasicVehicleContainerHighFrequency_out.speed);
		convert_DriveDirectiontoRos(_BasicVehicleContainerHighFrequency_in.driveDirection, _BasicVehicleContainerHighFrequency_out.driveDirection);
		convert_VehicleLengthtoRos(_BasicVehicleContainerHighFrequency_in.vehicleLength, _BasicVehicleContainerHighFrequency_out.vehicleLength);
		convert_VehicleWidthtoRos(_BasicVehicleContainerHighFrequency_in.vehicleWidth, _BasicVehicleContainerHighFrequency_out.vehicleWidth);
		convert_LongitudinalAccelerationtoRos(_BasicVehicleContainerHighFrequency_in.longitudinalAcceleration, _BasicVehicleContainerHighFrequency_out.longitudinalAcceleration);
		convert_CurvaturetoRos(_BasicVehicleContainerHighFrequency_in.curvature, _BasicVehicleContainerHighFrequency_out.curvature);
		convert_CurvatureCalculationModetoRos(_BasicVehicleContainerHighFrequency_in.curvatureCalculationMode, _BasicVehicleContainerHighFrequency_out.curvatureCalculationMode);
		convert_YawRatetoRos(_BasicVehicleContainerHighFrequency_in.yawRate, _BasicVehicleContainerHighFrequency_out.yawRate);
		if(_BasicVehicleContainerHighFrequency_in.accelerationControl)
		{
			convert_AccelerationControltoRos(*_BasicVehicleContainerHighFrequency_in.accelerationControl, _BasicVehicleContainerHighFrequency_out.accelerationControl);
			_BasicVehicleContainerHighFrequency_out.accelerationControl_isPresent = true;
		}
		if(_BasicVehicleContainerHighFrequency_in.lanePosition)
		{
			convert_LanePositiontoRos(*_BasicVehicleContainerHighFrequency_in.lanePosition, _BasicVehicleContainerHighFrequency_out.lanePosition);
			_BasicVehicleContainerHighFrequency_out.lanePosition_isPresent = true;
		}
		if(_BasicVehicleContainerHighFrequency_in.steeringWheelAngle)
		{
			convert_SteeringWheelAngletoRos(*_BasicVehicleContainerHighFrequency_in.steeringWheelAngle, _BasicVehicleContainerHighFrequency_out.steeringWheelAngle);
			_BasicVehicleContainerHighFrequency_out.steeringWheelAngle_isPresent = true;
		}
		if(_BasicVehicleContainerHighFrequency_in.lateralAcceleration)
		{
			convert_LateralAccelerationtoRos(*_BasicVehicleContainerHighFrequency_in.lateralAcceleration, _BasicVehicleContainerHighFrequency_out.lateralAcceleration);
			_BasicVehicleContainerHighFrequency_out.lateralAcceleration_isPresent = true;
		}
		if(_BasicVehicleContainerHighFrequency_in.verticalAcceleration)
		{
			convert_VerticalAccelerationtoRos(*_BasicVehicleContainerHighFrequency_in.verticalAcceleration, _BasicVehicleContainerHighFrequency_out.verticalAcceleration);
			_BasicVehicleContainerHighFrequency_out.verticalAcceleration_isPresent = true;
		}
		if(_BasicVehicleContainerHighFrequency_in.performanceClass)
		{
			convert_PerformanceClasstoRos(*_BasicVehicleContainerHighFrequency_in.performanceClass, _BasicVehicleContainerHighFrequency_out.performanceClass);
			_BasicVehicleContainerHighFrequency_out.performanceClass_isPresent = true;
		}
		if(_BasicVehicleContainerHighFrequency_in.cenDsrcTollingZone)
		{
			convert_CenDsrcTollingZonetoRos(*_BasicVehicleContainerHighFrequency_in.cenDsrcTollingZone, _BasicVehicleContainerHighFrequency_out.cenDsrcTollingZone);
			_BasicVehicleContainerHighFrequency_out.cenDsrcTollingZone_isPresent = true;
		}
	}
	void convert_BasicVehicleContainerHighFrequencytoC(const etsi_its_cam_msgs::BasicVehicleContainerHighFrequency& _BasicVehicleContainerHighFrequency_in, BasicVehicleContainerHighFrequency_t& _BasicVehicleContainerHighFrequency_out)
	{
		memset(&_BasicVehicleContainerHighFrequency_out, 0, sizeof(BasicVehicleContainerHighFrequency_t));
		convert_HeadingtoC(_BasicVehicleContainerHighFrequency_in.heading, _BasicVehicleContainerHighFrequency_out.heading);
		convert_SpeedtoC(_BasicVehicleContainerHighFrequency_in.speed, _BasicVehicleContainerHighFrequency_out.speed);
		convert_DriveDirectiontoC(_BasicVehicleContainerHighFrequency_in.driveDirection, _BasicVehicleContainerHighFrequency_out.driveDirection);
		convert_VehicleLengthtoC(_BasicVehicleContainerHighFrequency_in.vehicleLength, _BasicVehicleContainerHighFrequency_out.vehicleLength);
		convert_VehicleWidthtoC(_BasicVehicleContainerHighFrequency_in.vehicleWidth, _BasicVehicleContainerHighFrequency_out.vehicleWidth);
		convert_LongitudinalAccelerationtoC(_BasicVehicleContainerHighFrequency_in.longitudinalAcceleration, _BasicVehicleContainerHighFrequency_out.longitudinalAcceleration);
		convert_CurvaturetoC(_BasicVehicleContainerHighFrequency_in.curvature, _BasicVehicleContainerHighFrequency_out.curvature);
		convert_CurvatureCalculationModetoC(_BasicVehicleContainerHighFrequency_in.curvatureCalculationMode, _BasicVehicleContainerHighFrequency_out.curvatureCalculationMode);
		convert_YawRatetoC(_BasicVehicleContainerHighFrequency_in.yawRate, _BasicVehicleContainerHighFrequency_out.yawRate);
		if(_BasicVehicleContainerHighFrequency_in.accelerationControl_isPresent)
		{
			AccelerationControl_t accelerationControl;
			convert_AccelerationControltoC(_BasicVehicleContainerHighFrequency_in.accelerationControl, accelerationControl);
			_BasicVehicleContainerHighFrequency_out.accelerationControl = new AccelerationControl_t(accelerationControl);
		}
		if(_BasicVehicleContainerHighFrequency_in.lanePosition_isPresent)
		{
			LanePosition_t lanePosition;
			convert_LanePositiontoC(_BasicVehicleContainerHighFrequency_in.lanePosition, lanePosition);
			_BasicVehicleContainerHighFrequency_out.lanePosition = new LanePosition_t(lanePosition);
		}
		if(_BasicVehicleContainerHighFrequency_in.steeringWheelAngle_isPresent)
		{
			SteeringWheelAngle_t steeringWheelAngle;
			convert_SteeringWheelAngletoC(_BasicVehicleContainerHighFrequency_in.steeringWheelAngle, steeringWheelAngle);
			_BasicVehicleContainerHighFrequency_out.steeringWheelAngle = new SteeringWheelAngle_t(steeringWheelAngle);
		}
		if(_BasicVehicleContainerHighFrequency_in.lateralAcceleration_isPresent)
		{
			LateralAcceleration_t lateralAcceleration;
			convert_LateralAccelerationtoC(_BasicVehicleContainerHighFrequency_in.lateralAcceleration, lateralAcceleration);
			_BasicVehicleContainerHighFrequency_out.lateralAcceleration = new LateralAcceleration_t(lateralAcceleration);
		}
		if(_BasicVehicleContainerHighFrequency_in.verticalAcceleration_isPresent)
		{
			VerticalAcceleration_t verticalAcceleration;
			convert_VerticalAccelerationtoC(_BasicVehicleContainerHighFrequency_in.verticalAcceleration, verticalAcceleration);
			_BasicVehicleContainerHighFrequency_out.verticalAcceleration = new VerticalAcceleration_t(verticalAcceleration);
		}
		if(_BasicVehicleContainerHighFrequency_in.performanceClass_isPresent)
		{
			PerformanceClass_t performanceClass;
			convert_PerformanceClasstoC(_BasicVehicleContainerHighFrequency_in.performanceClass, performanceClass);
			_BasicVehicleContainerHighFrequency_out.performanceClass = new PerformanceClass_t(performanceClass);
		}
		if(_BasicVehicleContainerHighFrequency_in.cenDsrcTollingZone_isPresent)
		{
			CenDsrcTollingZone_t cenDsrcTollingZone;
			convert_CenDsrcTollingZonetoC(_BasicVehicleContainerHighFrequency_in.cenDsrcTollingZone, cenDsrcTollingZone);
			_BasicVehicleContainerHighFrequency_out.cenDsrcTollingZone = new CenDsrcTollingZone_t(cenDsrcTollingZone);
		}
	}
}