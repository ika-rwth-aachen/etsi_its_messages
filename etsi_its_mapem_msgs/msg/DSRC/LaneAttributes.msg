# ------------------------------------------------------------------------------
# LaneAttributes ::= SEQUENCE {
#   directionalUse  LaneDirection,
#   sharedWith      LaneSharing,
#   laneType        LaneTypeAttributes,
#   regional        RegionalExtension {{Reg-LaneAttributes}} OPTIONAL
# ------------------------------------------------------------------------------

LaneDirection directionalUse

LaneSharing sharedWith

LaneTypeAttributes laneType

# actual-parameters: ['{']
RegionalExtension regional
bool regional_isPresent
