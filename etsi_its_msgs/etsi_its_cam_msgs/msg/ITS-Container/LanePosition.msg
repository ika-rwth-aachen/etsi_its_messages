# ------------------------------------------------------------------------------
# LanePosition ::= INTEGER {offTheRoad(-1), innerHardShoulder(0),
# innermostDrivingLane(1), secondLaneFromInside(2), outerHardShoulder(14) } (-1..14)
# ------------------------------------------------------------------------------

int8 value
int8 MIN = -1
int8 MAX = 14
int8 OFF_THE_ROAD = -1
int8 INNER_HARD_SHOULDER = 0
int8 INNERMOST_DRIVING_LANE = 1
int8 SECOND_LANE_FROM_INSIDE = 2
int8 OUTER_HARD_SHOULDER = 14

