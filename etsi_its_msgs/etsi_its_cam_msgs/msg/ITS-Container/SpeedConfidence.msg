# ------------------------------------------------------------------------------
# SpeedConfidence ::= INTEGER {equalOrWithinOneCentimeterPerSec(1), equalOrWithinOneMeterPerSec(100), outOfRange(126), unavailable(127)} (1..127)
# ------------------------------------------------------------------------------

uint8 value
uint8 MIN = 1
uint8 MAX = 127
uint8 EQUAL_OR_WITHIN_ONE_CENTIMETER_PER_SEC = 1
uint8 EQUAL_OR_WITHIN_ONE_METER_PER_SEC = 100
uint8 OUT_OF_RANGE = 126
uint8 UNAVAILABLE = 127

