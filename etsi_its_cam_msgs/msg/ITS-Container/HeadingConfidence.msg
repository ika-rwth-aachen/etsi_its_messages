# ------------------------------------------------------------------------------
# HeadingConfidence ::= INTEGER {equalOrWithinZeroPointOneDegree (1), equalOrWithinOneDegree (10), outOfRange(126), unavailable(127)} (1..127)
# ------------------------------------------------------------------------------

# restricted-to: [(1, 127)]
int64 EQUAL_OR_WITHIN_ZERO_POINT_ONE_DEGREE = 1
int64 EQUAL_OR_WITHIN_ONE_DEGREE = 10
int64 OUT_OF_RANGE = 126
int64 UNAVAILABLE = 127
int64 value
