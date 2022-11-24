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

# optional: True
LanePosition lanePosition

# optional: True
SteeringWheelAngle steeringWheelAngle

# optional: True
LateralAcceleration lateralAcceleration

# optional: True
VerticalAcceleration verticalAcceleration

# optional: True
PerformanceClass performanceClass

# optional: True
CenDsrcTollingZone cenDsrcTollingZone
