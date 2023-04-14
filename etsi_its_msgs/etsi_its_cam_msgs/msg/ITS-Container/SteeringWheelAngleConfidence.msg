# ------------------------------------------------------------------------------
# SteeringWheelAngleConfidence ::= INTEGER {equalOrWithinOnePointFiveDegree (1), outOfRange(126), unavailable(127)} (1..127)
# ------------------------------------------------------------------------------

# restricted-to: [(1, 127)]
uint8 EQUAL_OR_WITHIN_ONE_POINT_FIVE_DEGREE = 1
uint8 OUT_OF_RANGE = 126
uint8 UNAVAILABLE = 127
uint8 value
