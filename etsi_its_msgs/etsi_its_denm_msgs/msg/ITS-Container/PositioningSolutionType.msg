# ------------------------------------------------------------------------------
# PositioningSolutionType ::= ENUMERATED {noPositioningSolution(0), sGNSS(1), dGNSS(2), sGNSSplusDR(3), dGNSSplusDR(4), dR(5), ...}
# ------------------------------------------------------------------------------

uint8 value
uint8 NO_POSITIONING_SOLUTION = 0
uint8 S_GNSS = 1
uint8 D_GNSS = 2
uint8 S_GNSS_PLUS_D_R = 3
uint8 D_GNSS_PLUS_D_R = 4
uint8 D_R = 5

