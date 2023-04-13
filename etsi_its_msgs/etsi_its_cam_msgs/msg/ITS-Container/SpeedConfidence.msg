# ------------------------------------------------------------------------------
# SpeedConfidence ::= INTEGER {equalOrWithinOneCentimeterPerSec(1), equalOrWithinOneMeterPerSec(100), outOfRange(126), unavailable(127)} (1..127)
# ------------------------------------------------------------------------------

int64 MIN = 1
int64 MAX = 127
int64 EQUAL_OR_WITHIN_ONE_CENTIMETER_PER_SEC = 1
int64 EQUAL_OR_WITHIN_ONE_METER_PER_SEC = 100
int64 OUT_OF_RANGE = 126
int64 UNAVAILABLE = 127
int64 value
