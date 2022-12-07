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

uint8 UNAVAILABLE = 0
uint8 PREC_1_0_0MS = 1
uint8 PREC_1_0MS = 2
uint8 PREC_5MS = 3
uint8 PREC_1MS = 4
uint8 PREC_0_1MS = 5
uint8 PREC_0_0_5MS = 6
uint8 PREC_0_0_1MS = 7
uint8 value
