# ------------------------------------------------------------------------------
# MapData ::= SEQUENCE {
#   timeStamp         MinuteOfTheYear OPTIONAL, 
#   msgIssueRevision  MsgCount,        
#   layerType         LayerType OPTIONAL,
#   layerID           LayerID  OPTIONAL,
#   intersections     IntersectionGeometryList OPTIONAL,
#   roadSegments      RoadSegmentList OPTIONAL,
#   dataParameters    DataParameters OPTIONAL,
#   restrictionList   RestrictionClassList OPTIONAL,
#   regional          SEQUENCE (SIZE(1..4)) OF 
#                     RegionalExtension {{Reg-MapData}} OPTIONAL,
# ------------------------------------------------------------------------------

# optional: True
MinuteOfTheYear timeStamp

MsgCount msgIssueRevision

# optional: True
LayerType layerType

# optional: True
LayerID layerID

# optional: True
IntersectionGeometryList intersections

# optional: True
RoadSegmentList roadSegments

# optional: True
DataParameters dataParameters

# optional: True
RestrictionClassList restrictionList

# size: [(1, 4)]
# optional: True
RegionalExtension[] array
