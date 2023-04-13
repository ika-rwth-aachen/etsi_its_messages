# ------------------------------------------------------------------------------
# SteeringWheelAngleValue ::= INTEGER {straight(0), onePointFiveDegreesToRight(-1), onePointFiveDegreesToLeft(1), unavailable(512)} (-511..512)
# ------------------------------------------------------------------------------

int64 MIN = -511
int64 MAX = 512
int64 STRAIGHT = 0
int64 ONE_POINT_FIVE_DEGREES_TO_RIGHT = -1
int64 ONE_POINT_FIVE_DEGREES_TO_LEFT = 1
int64 UNAVAILABLE = 512
int64 value
