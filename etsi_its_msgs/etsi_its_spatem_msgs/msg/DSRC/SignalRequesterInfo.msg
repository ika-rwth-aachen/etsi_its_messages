# ------------------------------------------------------------------------------
# SignalRequesterInfo ::= SEQUENCE {
#   id             VehicleID,
#   request        RequestID,
#   sequenceNumber MsgCount,
#   role           BasicVehicleRole OPTIONAL,
#   typeData       RequestorType OPTIONAL, 
#   ...
# }
# ------------------------------------------------------------------------------

VehicleID id

RequestID request

MsgCount sequenceNumber

# optional: True
BasicVehicleRole role

# optional: True
RequestorType typeData
