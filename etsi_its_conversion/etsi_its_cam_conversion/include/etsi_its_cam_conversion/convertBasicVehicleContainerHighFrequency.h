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
  toRos_DriveDirection(in.drive_direction, out.drive_direction);
  toRos_VehicleLength(in.vehicle_length, out.vehicle_length);
  toRos_VehicleWidth(in.vehicle_width, out.vehicle_width);
  toRos_LongitudinalAcceleration(in.longitudinal_acceleration, out.longitudinal_acceleration);
  toRos_Curvature(in.curvature, out.curvature);
  toRos_CurvatureCalculationMode(in.curvature_calculation_mode, out.curvature_calculation_mode);
  toRos_YawRate(in.yaw_rate, out.yaw_rate);
  if (in.acceleration_control) {
    toRos_AccelerationControl(*in.acceleration_control, out.acceleration_control);
    out.acceleration_control_is_present = true;
  }

  if (in.lane_position) {
    toRos_LanePosition(*in.lane_position, out.lane_position);
    out.lane_position_is_present = true;
  }

  if (in.steering_wheel_angle) {
    toRos_SteeringWheelAngle(*in.steering_wheel_angle, out.steering_wheel_angle);
    out.steering_wheel_angle_is_present = true;
  }

  if (in.lateral_acceleration) {
    toRos_LateralAcceleration(*in.lateral_acceleration, out.lateral_acceleration);
    out.lateral_acceleration_is_present = true;
  }

  if (in.vertical_acceleration) {
    toRos_VerticalAcceleration(*in.vertical_acceleration, out.vertical_acceleration);
    out.vertical_acceleration_is_present = true;
  }

  if (in.performance_class) {
    toRos_PerformanceClass(*in.performance_class, out.performance_class);
    out.performance_class_is_present = true;
  }

  if (in.cen_dsrc_tolling_zone) {
    toRos_CenDsrcTollingZone(*in.cen_dsrc_tolling_zone, out.cen_dsrc_tolling_zone);
    out.cen_dsrc_tolling_zone_is_present = true;
  }

}

void toStruct_BasicVehicleContainerHighFrequency(const etsi_its_cam_msgs::BasicVehicleContainerHighFrequency& in, BasicVehicleContainerHighFrequency_t& out) {
    
  memset(&out, 0, sizeof(BasicVehicleContainerHighFrequency_t));

  toStruct_Heading(in.heading, out.heading);
  toStruct_Speed(in.speed, out.speed);
  toStruct_DriveDirection(in.drive_direction, out.drive_direction);
  toStruct_VehicleLength(in.vehicle_length, out.vehicle_length);
  toStruct_VehicleWidth(in.vehicle_width, out.vehicle_width);
  toStruct_LongitudinalAcceleration(in.longitudinal_acceleration, out.longitudinal_acceleration);
  toStruct_Curvature(in.curvature, out.curvature);
  toStruct_CurvatureCalculationMode(in.curvature_calculation_mode, out.curvature_calculation_mode);
  toStruct_YawRate(in.yaw_rate, out.yaw_rate);
  if (in.acceleration_control_is_present) {
    AccelerationControl_t acceleration_control;
    toStruct_AccelerationControl(in.acceleration_control, acceleration_control);
    out.acceleration_control = new AccelerationControl_t(acceleration_control);
  }

  if (in.lane_position_is_present) {
    LanePosition_t lane_position;
    toStruct_LanePosition(in.lane_position, lane_position);
    out.lane_position = new LanePosition_t(lane_position);
  }

  if (in.steering_wheel_angle_is_present) {
    SteeringWheelAngle_t steering_wheel_angle;
    toStruct_SteeringWheelAngle(in.steering_wheel_angle, steering_wheel_angle);
    out.steering_wheel_angle = new SteeringWheelAngle_t(steering_wheel_angle);
  }

  if (in.lateral_acceleration_is_present) {
    LateralAcceleration_t lateral_acceleration;
    toStruct_LateralAcceleration(in.lateral_acceleration, lateral_acceleration);
    out.lateral_acceleration = new LateralAcceleration_t(lateral_acceleration);
  }

  if (in.vertical_acceleration_is_present) {
    VerticalAcceleration_t vertical_acceleration;
    toStruct_VerticalAcceleration(in.vertical_acceleration, vertical_acceleration);
    out.vertical_acceleration = new VerticalAcceleration_t(vertical_acceleration);
  }

  if (in.performance_class_is_present) {
    PerformanceClass_t performance_class;
    toStruct_PerformanceClass(in.performance_class, performance_class);
    out.performance_class = new PerformanceClass_t(performance_class);
  }

  if (in.cen_dsrc_tolling_zone_is_present) {
    CenDsrcTollingZone_t cen_dsrc_tolling_zone;
    toStruct_CenDsrcTollingZone(in.cen_dsrc_tolling_zone, cen_dsrc_tolling_zone);
    out.cen_dsrc_tolling_zone = new CenDsrcTollingZone_t(cen_dsrc_tolling_zone);
  }

}

}