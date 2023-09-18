# ------------------------------------------------------------------------------
# BasicVehicleContainerHighFrequency ::= SEQUENCE {
# 	heading Heading,
# 	speed Speed,
# 	driveDirection DriveDirection,
# 	vehicleLength VehicleLength,
# 	vehicleWidth VehicleWidth,
# 	longitudinalAcceleration LongitudinalAcceleration,
# 	curvature Curvature,
# 	curvatureCalculationMode CurvatureCalculationMode,
# 	yawRate YawRate,
# 	accelerationControl AccelerationControl OPTIONAL,
# 	lanePosition LanePosition OPTIONAL,
# 	steeringWheelAngle SteeringWheelAngle OPTIONAL,
# 	lateralAcceleration LateralAcceleration OPTIONAL,
# 	verticalAcceleration VerticalAcceleration OPTIONAL,
# 	performanceClass PerformanceClass OPTIONAL,
# 	cenDsrcTollingZone CenDsrcTollingZone OPTIONAL
# }
# ------------------------------------------------------------------------------

Heading heading

Speed speed

DriveDirection drive_direction

VehicleLength vehicle_length

VehicleWidth vehicle_width

LongitudinalAcceleration longitudinal_acceleration

Curvature curvature

CurvatureCalculationMode curvature_calculation_mode

YawRate yaw_rate

AccelerationControl acceleration_control
bool acceleration_control_is_present

LanePosition lane_position
bool lane_position_is_present

SteeringWheelAngle steering_wheel_angle
bool steering_wheel_angle_is_present

LateralAcceleration lateral_acceleration
bool lateral_acceleration_is_present

VerticalAcceleration vertical_acceleration
bool vertical_acceleration_is_present

PerformanceClass performance_class
bool performance_class_is_present

CenDsrcTollingZone cen_dsrc_tolling_zone
bool cen_dsrc_tolling_zone_is_present

