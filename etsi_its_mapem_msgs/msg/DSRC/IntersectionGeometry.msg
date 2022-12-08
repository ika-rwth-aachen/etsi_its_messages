# ------------------------------------------------------------------------------
# IntersectionGeometry ::= SEQUENCE {
#   name        DescriptiveName OPTIONAL, 
#   id          IntersectionReferenceID,  
#   revision    MsgCount,    
#   refPoint    Position3D,
#   laneWidth   LaneWidth OPTIONAL,   
#   speedLimits SpeedLimitList OPTIONAL, 
#   laneSet     LaneList,
#   preemptPriorityData PreemptPriorityList OPTIONAL, 
#   regional     SEQUENCE (SIZE(1..4)) OF 
#                RegionalExtension {{Reg-IntersectionGeometry}} OPTIONAL,
# ------------------------------------------------------------------------------

DescriptiveName name
bool name_isPresent

IntersectionReferenceID id

MsgCount revision

Position3D refPoint

LaneWidth laneWidth
bool laneWidth_isPresent

SpeedLimitList speedLimits
bool speedLimits_isPresent

LaneList laneSet

PreemptPriorityList preemptPriorityData
bool preemptPriorityData_isPresent

# size: [(1, 4)]
RegionalExtension[] regional
bool regional_isPresent
