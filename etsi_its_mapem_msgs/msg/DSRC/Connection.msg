# ------------------------------------------------------------------------------
# Connection ::= SEQUENCE {
#   connectingLane     ConnectingLane, 
#   remoteIntersection IntersectionReferenceID OPTIONAL,  
#   signalGroup        SignalGroupID OPTIONAL,    
#   userClass          RestrictionClassID OPTIONAL, 
#   connectionID       LaneConnectionID OPTIONAL
# }
# ------------------------------------------------------------------------------

ConnectingLane connectingLane

IntersectionReferenceID remoteIntersection
bool remoteIntersection_isPresent

SignalGroupID signalGroup
bool signalGroup_isPresent

RestrictionClassID userClass
bool userClass_isPresent

LaneConnectionID connectionID
bool connectionID_isPresent
