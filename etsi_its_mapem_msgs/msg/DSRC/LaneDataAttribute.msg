# ------------------------------------------------------------------------------
# LaneDataAttribute ::= CHOICE {
#    pathEndPointAngle        DeltaAngle,  
#    laneCrownPointCenter     RoadwayCrownAngle,  
#    laneCrownPointLeft       RoadwayCrownAngle,  
#    laneCrownPointRight      RoadwayCrownAngle,  
#    laneAngle                MergeDivergeNodeAngle,  
#    speedLimits              SpeedLimitList,
#    regional  SEQUENCE (SIZE(1..4)) OF 
#              RegionalExtension {{Reg-LaneDataAttribute}}, 
# ------------------------------------------------------------------------------

uint8 choice

DeltaAngle pathEndPointAngle
uint8 CHOICE_PATH_END_POINT_ANGLE = 0
RoadwayCrownAngle laneCrownPointCenter
uint8 CHOICE_LANE_CROWN_POINT_CENTER = 1
RoadwayCrownAngle laneCrownPointLeft
uint8 CHOICE_LANE_CROWN_POINT_LEFT = 2
RoadwayCrownAngle laneCrownPointRight
uint8 CHOICE_LANE_CROWN_POINT_RIGHT = 3
MergeDivergeNodeAngle laneAngle
uint8 CHOICE_LANE_ANGLE = 4
SpeedLimitList speedLimits
uint8 CHOICE_SPEED_LIMITS = 5
# size: [(1, 4)]
RegionalExtension[] array
uint8 CHOICE_REGIONAL = 6
