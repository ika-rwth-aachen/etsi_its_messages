# ------------------------------------------------------------------------------
# RequestorType ::= SEQUENCE {
#   role         BasicVehicleRole,
#   subrole      RequestSubRole OPTIONAL,
#   request      RequestImportanceLevel OPTIONAL,
#   iso3883      Iso3833VehicleType OPTIONAL,
#   hpmsType     VehicleType OPTIONAL,
#   regional     RegionalExtension {{Reg-RequestorType}} OPTIONAL,
# ------------------------------------------------------------------------------

BasicVehicleRole role

# optional: True
RequestSubRole subrole

# optional: True
RequestImportanceLevel request

# optional: True
Iso3833VehicleType iso3883

# optional: True
VehicleType hpmsType

# actual-parameters: ['{']
# optional: True
RegionalExtension regional
