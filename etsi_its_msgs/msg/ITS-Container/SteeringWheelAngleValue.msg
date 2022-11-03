# ------------------------------------------------------------------------------
# SteeringWheelAngleValue ::= INTEGER {straight(0), onePointFiveDegreesToRight(-1), onePointFiveDegreesToLeft(1), unavailable(512)} (-511..512)
# ------------------------------------------------------------------------------

# restricted-to: [(-511, 512)]
int16 STRAIGHT = 0
int16 ONE_POINT_FIVE_DEGREES_TO_RIGHT = -1
int16 ONE_POINT_FIVE_DEGREES_TO_LEFT = 1
int16 UNAVAILABLE = 512
int16 value
