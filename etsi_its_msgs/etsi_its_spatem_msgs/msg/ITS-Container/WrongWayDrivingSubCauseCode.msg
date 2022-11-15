# ------------------------------------------------------------------------------
# WrongWayDrivingSubCauseCode ::= INTEGER {unavailable(0), wrongLane(1), wrongDirection(2)} (0..255)
# ------------------------------------------------------------------------------

# restricted-to: [(0, 255)]
uint8 UNAVAILABLE = 0
uint8 WRONG_LANE = 1
uint8 WRONG_DIRECTION = 2
uint8 value
