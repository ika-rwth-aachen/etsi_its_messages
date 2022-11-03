# ------------------------------------------------------------------------------
# RoadSegment ::= SEQUENCE {
#   name        DescriptiveName OPTIONAL,
#   id          RoadSegmentReferenceID,
#   revision    MsgCount,    
#   refPoint    Position3D,
#   laneWidth   LaneWidth OPTIONAL,   
#   speedLimits SpeedLimitList OPTIONAL, 
#   roadLaneSet RoadLaneSetList,   
#   regional    SEQUENCE (SIZE(1..4)) OF 
#               RegionalExtension {{Reg-RoadSegment}} OPTIONAL,
# ------------------------------------------------------------------------------

# optional: True
DescriptiveName name

RoadSegmentReferenceID id

MsgCount revision

Position3D refPoint

# optional: True
LaneWidth laneWidth

# optional: True
SpeedLimitList speedLimits

RoadLaneSetList roadLaneSet

# size: [(1, 4)]
# optional: True
RegionalExtension[] array
