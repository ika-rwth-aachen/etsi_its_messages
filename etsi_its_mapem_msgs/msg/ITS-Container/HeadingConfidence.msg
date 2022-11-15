# ------------------------------------------------------------------------------
# HeadingConfidence ::= ENUMERATED {
#    unavailable   (0),
#    prec10deg     (1),
#    prec05deg     (2),
#    prec01deg     (3),
#    prec0-1deg    (4),
#    prec0-05deg   (5),
#    prec0-01deg   (6),
#    prec0-0125deg (7)
#    } 
# ------------------------------------------------------------------------------

# restricted-to: [(1, 127)]
uint8 EQUAL_OR_WITHIN_ZERO_POINT_ONE_DEGREE = 1
uint8 EQUAL_OR_WITHIN_ONE_DEGREE = 10
uint8 OUT_OF_RANGE = 126
uint8 UNAVAILABLE = 127
uint8 value
