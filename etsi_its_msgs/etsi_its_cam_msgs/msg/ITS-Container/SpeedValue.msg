# ------------------------------------------------------------------------------
# SpeedValue ::= INTEGER {standstill(0), oneCentimeterPerSec(1), unavailable(16383)} (0..16383)
# ------------------------------------------------------------------------------

# restricted-to: [(0, 16383)]
uint16 STANDSTILL = 0
uint16 ONE_CENTIMETER_PER_SEC = 1
uint16 UNAVAILABLE = 16383
uint16 value
