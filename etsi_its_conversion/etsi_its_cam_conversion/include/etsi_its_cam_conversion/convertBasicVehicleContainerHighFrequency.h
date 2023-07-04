#pragma once

#include <etsi_its_cam_coding/BasicVehicleContainerHighFrequency.h>
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
#include <etsi_its_cam_msgs/BasicVehicleContainerHighFrequency.h>


namespace etsi_its_cam_conversion {

void toRos_BasicVehicleContainerHighFrequency(const BasicVehicleContainerHighFrequency_t& in, etsi_its_cam_msgs::BasicVehicleContainerHighFrequency& out) {

  toRos_Heading(in.heading, out.heading);
  toRos_Speed(in.speed, out.speed);
  toRos_DriveDirection(in.driveDirection, out.driveDirection);
  toRos_VehicleLength(in.vehicleLength, out.vehicleLength);
  toRos_VehicleWidth(in.vehicleWidth, out.vehicleWidth);
  toRos_LongitudinalAcceleration(in.longitudinalAcceleration, out.longitudinalAcceleration);
  toRos_Curvature(in.curvature, out.curvature);
  toRos_CurvatureCalculationMode(in.curvatureCalculationMode, out.curvatureCalculationMode);
  toRos_YawRate(in.yawRate, out.yawRate);
  if (in.accelerationControl) {
    toRos_AccelerationControl(*in.accelerationControl, out.accelerationControl);
    out.accelerationControl_isPresent = true;
  }

  if (in.lanePosition) {
    toRos_LanePosition(*in.lanePosition, out.lanePosition);
    out.lanePosition_isPresent = true;
  }

  if (in.steeringWheelAngle) {
    toRos_SteeringWheelAngle(*in.steeringWheelAngle, out.steeringWheelAngle);
    out.steeringWheelAngle_isPresent = true;
  }

  if (in.lateralAcceleration) {
    toRos_LateralAcceleration(*in.lateralAcceleration, out.lateralAcceleration);
    out.lateralAcceleration_isPresent = true;
  }

  if (in.verticalAcceleration) {
    toRos_VerticalAcceleration(*in.verticalAcceleration, out.verticalAcceleration);
    out.verticalAcceleration_isPresent = true;
  }

  if (in.performanceClass) {
    toRos_PerformanceClass(*in.performanceClass, out.performanceClass);
    out.performanceClass_isPresent = true;
  }

  if (in.cenDsrcTollingZone) {
    toRos_CenDsrcTollingZone(*in.cenDsrcTollingZone, out.cenDsrcTollingZone);
    out.cenDsrcTollingZone_isPresent = true;
  }

}

void toStruct_BasicVehicleContainerHighFrequency(const etsi_its_cam_msgs::BasicVehicleContainerHighFrequency& in, BasicVehicleContainerHighFrequency_t& out) {
    
  memset(&out, 0, sizeof(BasicVehicleContainerHighFrequency_t));

  toStruct_Heading(in.heading, out.heading);
  toStruct_Speed(in.speed, out.speed);
  toStruct_DriveDirection(in.driveDirection, out.driveDirection);
  toStruct_VehicleLength(in.vehicleLength, out.vehicleLength);
  toStruct_VehicleWidth(in.vehicleWidth, out.vehicleWidth);
  toStruct_LongitudinalAcceleration(in.longitudinalAcceleration, out.longitudinalAcceleration);
  toStruct_Curvature(in.curvature, out.curvature);
  toStruct_CurvatureCalculationMode(in.curvatureCalculationMode, out.curvatureCalculationMode);
  toStruct_YawRate(in.yawRate, out.yawRate);
  if (in.accelerationControl_isPresent) {
    AccelerationControl_t accelerationControl;
    toStruct_AccelerationControl(in.accelerationControl, accelerationControl);
    out.accelerationControl = new AccelerationControl_t(accelerationControl);
  }

  if (in.lanePosition_isPresent) {
    LanePosition_t lanePosition;
    toStruct_LanePosition(in.lanePosition, lanePosition);
    out.lanePosition = new LanePosition_t(lanePosition);
  }

  if (in.steeringWheelAngle_isPresent) {
    SteeringWheelAngle_t steeringWheelAngle;
    toStruct_SteeringWheelAngle(in.steeringWheelAngle, steeringWheelAngle);
    out.steeringWheelAngle = new SteeringWheelAngle_t(steeringWheelAngle);
  }

  if (in.lateralAcceleration_isPresent) {
    LateralAcceleration_t lateralAcceleration;
    toStruct_LateralAcceleration(in.lateralAcceleration, lateralAcceleration);
    out.lateralAcceleration = new LateralAcceleration_t(lateralAcceleration);
  }

  if (in.verticalAcceleration_isPresent) {
    VerticalAcceleration_t verticalAcceleration;
    toStruct_VerticalAcceleration(in.verticalAcceleration, verticalAcceleration);
    out.verticalAcceleration = new VerticalAcceleration_t(verticalAcceleration);
  }

  if (in.performanceClass_isPresent) {
    PerformanceClass_t performanceClass;
    toStruct_PerformanceClass(in.performanceClass, performanceClass);
    out.performanceClass = new PerformanceClass_t(performanceClass);
  }

  if (in.cenDsrcTollingZone_isPresent) {
    CenDsrcTollingZone_t cenDsrcTollingZone;
    toStruct_CenDsrcTollingZone(in.cenDsrcTollingZone, cenDsrcTollingZone);
    out.cenDsrcTollingZone = new CenDsrcTollingZone_t(cenDsrcTollingZone);
  }

}

}