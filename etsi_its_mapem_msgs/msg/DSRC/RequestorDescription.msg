# ------------------------------------------------------------------------------
# RequestorDescription ::= SEQUENCE {
#   id                VehicleID, 
#   type              RequestorType OPTIONAL,
#   position          RequestorPositionVector OPTIONAL,
#   name              DescriptiveName OPTIONAL,
#   routeName         DescriptiveName OPTIONAL,
#   transitStatus     TransitVehicleStatus OPTIONAL,
#   transitOccupancy  TransitVehicleOccupancy OPTIONAL,
#   transitSchedule   DeltaTime OPTIONAL, 
#   regional          SEQUENCE (SIZE(1..4)) OF 
#                     RegionalExtension {{Reg-RequestorDescription}} OPTIONAL,
# ------------------------------------------------------------------------------

VehicleID id

# optional: True
RequestorType type

# optional: True
RequestorPositionVector position

# optional: True
DescriptiveName name

# optional: True
DescriptiveName routeName

# optional: True
TransitVehicleStatus transitStatus

# optional: True
TransitVehicleOccupancy transitOccupancy

# optional: True
DeltaTime transitSchedule

# size: [(1, 4)]
# optional: True
RegionalExtension[] array
