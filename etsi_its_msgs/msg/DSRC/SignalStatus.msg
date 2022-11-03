# ------------------------------------------------------------------------------
# SignalStatus ::= SEQUENCE {
#   sequenceNumber MsgCount,
#   id             IntersectionReferenceID,
#   sigStatus      SignalStatusPackageList,
#   regional       SEQUENCE (SIZE(1..4)) OF 
#                  RegionalExtension {{Reg-SignalStatus}} OPTIONAL,
# ------------------------------------------------------------------------------

MsgCount sequenceNumber

IntersectionReferenceID id

SignalStatusPackageList sigStatus

# size: [(1, 4)]
# optional: True
RegionalExtension[] array
