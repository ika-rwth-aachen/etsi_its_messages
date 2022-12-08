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

# optional: True
AccelerationControl accelerationControl
bool accelerationControl_isPresent

# optional: True
LanePosition lanePosition
bool lanePosition_isPresent

# optional: True
SteeringWheelAngle steeringWheelAngle
bool steeringWheelAngle_isPresent

# optional: True
LateralAcceleration lateralAcceleration
bool lateralAcceleration_isPresent

# optional: True
VerticalAcceleration verticalAcceleration
bool verticalAcceleration_isPresent

# optional: True
PerformanceClass performanceClass
bool performanceClass_isPresent

# optional: True
CenDsrcTollingZone cenDsrcTollingZone
bool cenDsrcTollingZone_isPresent
