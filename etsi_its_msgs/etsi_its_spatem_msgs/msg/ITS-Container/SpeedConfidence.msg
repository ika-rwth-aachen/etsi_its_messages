# ------------------------------------------------------------------------------
# SpeedConfidence ::= ENUMERATED {
#    unavailable (0), -- Not Equipped or unavailable
#    prec100ms   (1), -- 100  meters / sec
#    prec10ms    (2), -- 10   meters / sec
#    prec5ms     (3), -- 5    meters / sec
#    prec1ms     (4), -- 1    meters / sec
#    prec0-1ms   (5), -- 0.1  meters / sec
#    prec0-05ms  (6), -- 0.05 meters / sec
#    prec0-01ms  (7)  -- 0.01 meters / sec
#    }
# ------------------------------------------------------------------------------

# restricted-to: [(1, 127)]
uint8 EQUAL_OR_WITHIN_ONE_CENTIMETER_PER_SEC = 1
uint8 EQUAL_OR_WITHIN_ONE_METER_PER_SEC = 100
uint8 OUT_OF_RANGE = 126
uint8 UNAVAILABLE = 127
uint8 value
