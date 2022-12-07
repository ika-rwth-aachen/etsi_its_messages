# ------------------------------------------------------------------------------
# GenericLane ::= SEQUENCE { 
#   laneID           LaneID, 
#   name             DescriptiveName OPTIONAL,        
#   ingressApproach  ApproachID OPTIONAL,
#   egressApproach   ApproachID OPTIONAL,
#   laneAttributes   LaneAttributes, 
#   maneuvers        AllowedManeuvers OPTIONAL,
#   nodeList         NodeListXY,
#   connectsTo       ConnectsToList OPTIONAL, 
#   overlays         OverlayLaneList OPTIONAL, 
#   regional  SEQUENCE (SIZE(1..4)) OF 
#             RegionalExtension {{Reg-GenericLane}} OPTIONAL,
# ------------------------------------------------------------------------------

LaneID laneID

DescriptiveName name
bool name_isPresent

ApproachID ingressApproach
bool ingressApproach_isPresent

ApproachID egressApproach
bool egressApproach_isPresent

LaneAttributes laneAttributes

AllowedManeuvers maneuvers
bool maneuvers_isPresent

NodeListXY nodeList

ConnectsToList connectsTo
bool connectsTo_isPresent

OverlayLaneList overlays
bool overlays_isPresent

# size: [(1, 4)]
RegionalExtension[] array
bool regional_isPresent
