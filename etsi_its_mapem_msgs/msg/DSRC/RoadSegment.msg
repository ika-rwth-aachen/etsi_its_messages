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

DescriptiveName name
bool name_isPresent

RoadSegmentReferenceID id

MsgCount revision

Position3D refPoint

LaneWidth laneWidth
bool laneWidth_isPresent

SpeedLimitList speedLimits
bool speedLimits_isPresent

RoadLaneSetList roadLaneSet

# size: [(1, 4)]
RegionalExtension[] array
bool regional_isPresent
