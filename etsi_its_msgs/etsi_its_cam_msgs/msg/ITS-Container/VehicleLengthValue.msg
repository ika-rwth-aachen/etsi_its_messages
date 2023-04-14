# ------------------------------------------------------------------------------
# VehicleLengthValue ::= INTEGER {tenCentimeters(1), outOfRange(1022), unavailable(1023)}  (1..1023)
# ------------------------------------------------------------------------------

# restricted-to: [(1, 1023)]
uint16 TEN_CENTIMETERS = 1
uint16 OUT_OF_RANGE = 1022
uint16 UNAVAILABLE = 1023
uint16 value
