# ------------------------------------------------------------------------------
# SignalRequest ::= SEQUENCE {
#   id            IntersectionReferenceID,  
#   requestID     RequestID, 
#   requestType   PriorityRequestType,   
#   inBoundLane   IntersectionAccessPoint,       
#   outBoundLane  IntersectionAccessPoint OPTIONAL,       
#   regional      SEQUENCE (SIZE(1..4)) OF 
#                 RegionalExtension {{Reg-SignalRequest}} OPTIONAL,
# ------------------------------------------------------------------------------

IntersectionReferenceID id

RequestID requestID

PriorityRequestType requestType

IntersectionAccessPoint inBoundLane

# optional: True
IntersectionAccessPoint outBoundLane

# size: [(1, 4)]
# optional: True
RegionalExtension[] array
