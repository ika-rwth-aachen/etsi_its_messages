# ------------------------------------------------------------------------------
# Node ::= SEQUENCE {
#   id              INTEGER,
#   lane            LaneID           OPTIONAL,
#   connectionID    LaneConnectionID OPTIONAL,
#   intersectionID  IntersectionID   OPTIONAL,
#   ...
# }
# ------------------------------------------------------------------------------

int32 id

# optional: True
LaneID lane

# optional: True
LaneConnectionID connectionID

# optional: True
IntersectionID intersectionID
