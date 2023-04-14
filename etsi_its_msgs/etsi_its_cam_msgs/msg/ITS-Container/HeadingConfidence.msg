# ------------------------------------------------------------------------------
# HeadingConfidence ::= INTEGER {equalOrWithinZeroPointOneDegree (1), equalOrWithinOneDegree (10), outOfRange(126), unavailable(127)} (1..127)
# ------------------------------------------------------------------------------

# restricted-to: [(1, 127)]
uint8 EQUAL_OR_WITHIN_ZERO_POINT_ONE_DEGREE = 1
uint8 EQUAL_OR_WITHIN_ONE_DEGREE = 10
uint8 OUT_OF_RANGE = 126
uint8 UNAVAILABLE = 127
uint8 value
