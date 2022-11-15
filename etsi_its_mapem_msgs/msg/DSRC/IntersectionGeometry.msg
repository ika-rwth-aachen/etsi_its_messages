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

# optional: True
DescriptiveName name

IntersectionReferenceID id

MsgCount revision

Position3D refPoint

# optional: True
LaneWidth laneWidth

# optional: True
SpeedLimitList speedLimits

LaneList laneSet

# optional: True
PreemptPriorityList preemptPriorityData

# size: [(1, 4)]
# optional: True
RegionalExtension[] array
