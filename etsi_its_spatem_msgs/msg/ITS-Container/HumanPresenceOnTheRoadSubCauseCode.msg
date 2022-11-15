# ------------------------------------------------------------------------------
# HumanPresenceOnTheRoadSubCauseCode ::= INTEGER {unavailable(0), childrenOnRoadway(1), cyclistOnRoadway(2), motorcyclistOnRoadway(3)} (0..255)
# ------------------------------------------------------------------------------

# restricted-to: [(0, 255)]
uint8 UNAVAILABLE = 0
uint8 CHILDREN_ON_ROADWAY = 1
uint8 CYCLIST_ON_ROADWAY = 2
uint8 MOTORCYCLIST_ON_ROADWAY = 3
uint8 value
