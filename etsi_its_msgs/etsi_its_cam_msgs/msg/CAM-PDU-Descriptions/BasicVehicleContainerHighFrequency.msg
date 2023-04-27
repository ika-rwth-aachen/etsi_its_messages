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

DriveDirection driveDirection

VehicleLength vehicleLength

VehicleWidth vehicleWidth

LongitudinalAcceleration longitudinalAcceleration

Curvature curvature

CurvatureCalculationMode curvatureCalculationMode

YawRate yawRate

AccelerationControl accelerationControl
bool accelerationControl_isPresent

LanePosition lanePosition
bool lanePosition_isPresent

SteeringWheelAngle steeringWheelAngle
bool steeringWheelAngle_isPresent

LateralAcceleration lateralAcceleration
bool lateralAcceleration_isPresent

VerticalAcceleration verticalAcceleration
bool verticalAcceleration_isPresent

PerformanceClass performanceClass
bool performanceClass_isPresent

CenDsrcTollingZone cenDsrcTollingZone
bool cenDsrcTollingZone_isPresent

