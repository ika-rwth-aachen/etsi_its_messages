# ------------------------------------------------------------------------------
# NodeListXY ::= CHOICE {
#   nodes     NodeSetXY,
#   computed  ComputedLane,
#   ...
# }
# ------------------------------------------------------------------------------

uint8 choice

NodeSetXY nodes
uint8 CHOICE_NODES = 0
ComputedLane computed
uint8 CHOICE_COMPUTED = 1
