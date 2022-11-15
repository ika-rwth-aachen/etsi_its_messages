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

# optional: True
IntersectionReferenceID remoteIntersection

# optional: True
SignalGroupID signalGroup

# optional: True
RestrictionClassID userClass

# optional: True
LaneConnectionID connectionID
