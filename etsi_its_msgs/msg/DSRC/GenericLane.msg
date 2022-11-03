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

# optional: True
DescriptiveName name

# optional: True
ApproachID ingressApproach

# optional: True
ApproachID egressApproach

LaneAttributes laneAttributes

# optional: True
AllowedManeuvers maneuvers

NodeListXY nodeList

# optional: True
ConnectsToList connectsTo

# optional: True
OverlayLaneList overlays

# size: [(1, 4)]
# optional: True
RegionalExtension[] array
