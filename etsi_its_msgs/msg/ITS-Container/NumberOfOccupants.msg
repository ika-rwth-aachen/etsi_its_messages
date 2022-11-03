# ------------------------------------------------------------------------------
# NumberOfOccupants ::= INTEGER {oneOccupant (1), unavailable(127)} (0 .. 127)
# ------------------------------------------------------------------------------

# restricted-to: [(0, 127)]
uint8 ONE_OCCUPANT = 1
uint8 UNAVAILABLE = 127
uint8 value
