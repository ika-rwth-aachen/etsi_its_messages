Heading heading

Speed speed

int32 driveDirection
int32 DRIVE_DIRECTION_FORWARD = 0
int32 DRIVE_DIRECTION_BACKWARD = 1
int32 DRIVE_DIRECTION_UNAVAILABLE = 2

VehicleLength vehicleLength

# restricted-to: [(1, 62)]
int32 vehicleWidth
int32 VEHICLE_WIDTH_TEN_CENTIMETERS = 1
int32 VEHICLE_WIDTH_OUT_OF_RANGE = 61
int32 VEHICLE_WIDTH_UNAVAILABLE = 62

LongitudinalAcceleration longitudinalAcceleration

Curvature curvature

int32 curvatureCalculationMode
int32 CURVATURE_CALCULATION_MODE_YAW_RATE_USED = 0
int32 CURVATURE_CALCULATION_MODE_YAW_RATE_NOT_USED = 1
int32 CURVATURE_CALCULATION_MODE_UNAVAILABLE = 2

YawRate yawRate

# named-bits: [('brakePedalEngaged', '0'), ('gasPedalEngaged', '1'), ('emergencyBrakeEngaged', '2'), ('collisionWarningEngaged', '3'), ('accEngaged', '4'), ('cruiseControlEngaged', '5'), ('speedLimiterEngaged', '6')]
# size: [7]
string accelerationControl

# restricted-to: [(-1, 14)]
int32 lanePosition
int32 LANE_POSITION_OFF_THE_ROAD = -1
int32 LANE_POSITION_INNER_HARD_SHOULDER = 0
int32 LANE_POSITION_INNERMOST_DRIVING_LANE = 1
int32 LANE_POSITION_SECOND_LANE_FROM_INSIDE = 2
int32 LANE_POSITION_OUTER_HARD_SHOULDER = 14

# optional: True
SteeringWheelAngle steeringWheelAngle

# optional: True
LateralAcceleration lateralAcceleration

# optional: True
VerticalAcceleration verticalAcceleration

# restricted-to: [(0, 7)]
int32 performanceClass
int32 PERFORMANCE_CLASS_UNAVAILABLE = 0
int32 PERFORMANCE_CLASS_PERFORMANCE_CLASS_A = 1
int32 PERFORMANCE_CLASS_PERFORMANCE_CLASS_B = 2

# optional: True
CenDsrcTollingZone cenDsrcTollingZone

