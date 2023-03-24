# ------------------------------------------------------------------------------
# LanePosition ::= INTEGER {offTheRoad(-1), innerHardShoulder(0),
# innermostDrivingLane(1), secondLaneFromInside(2), outerHardShoulder(14) } (-1..14)
# ------------------------------------------------------------------------------

# restricted-to: [(-1, 14)]
int64 OFF_THE_ROAD = -1
int64 INNER_HARD_SHOULDER = 0
int64 INNERMOST_DRIVING_LANE = 1
int64 SECOND_LANE_FROM_INSIDE = 2
int64 OUTER_HARD_SHOULDER = 14
int64 value
