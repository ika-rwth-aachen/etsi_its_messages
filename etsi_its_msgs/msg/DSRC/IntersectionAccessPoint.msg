# ------------------------------------------------------------------------------
# IntersectionAccessPoint ::= CHOICE {
#   lane       LaneID,
#   approach   ApproachID,
#   connection LaneConnectionID,
#   ...
# }
# ------------------------------------------------------------------------------

uint8 choice

LaneID lane
uint8 CHOICE_LANE = 0
ApproachID approach
uint8 CHOICE_APPROACH = 1
LaneConnectionID connection
uint8 CHOICE_CONNECTION = 2
