# ------------------------------------------------------------------------------
# CurvatureValue ::= INTEGER {straight(0), reciprocalOf1MeterRadiusToRight(-30000), reciprocalOf1MeterRadiusToLeft(30000), unavailable(30001)} (-30000..30001)
# ------------------------------------------------------------------------------

# restricted-to: [(-30000, 30001)]
int16 STRAIGHT = 0
int16 RECIPROCAL_OF_1_METER_RADIUS_TO_RIGHT = -30000
int16 RECIPROCAL_OF_1_METER_RADIUS_TO_LEFT = 30000
int16 UNAVAILABLE = 30001
int16 value
