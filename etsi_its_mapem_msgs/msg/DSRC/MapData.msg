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

MinuteOfTheYear timeStamp
bool timeStamp_isPresent

MsgCount msgIssueRevision

LayerType layerType
bool layerType_isPresent

LayerID layerID
bool layerID_isPresent

IntersectionGeometryList intersections
bool intersections_isPresent

RoadSegmentList roadSegments
bool roadSegments_isPresent

DataParameters dataParameters
bool dataParameters_isPresent

RestrictionClassList restrictionList
bool restrictionList_isPresent

# size: [(1, 4)]
RegionalExtension[] regional
bool regional_isPresent
