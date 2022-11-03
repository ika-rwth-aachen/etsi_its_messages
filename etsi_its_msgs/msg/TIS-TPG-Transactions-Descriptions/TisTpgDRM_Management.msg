# ------------------------------------------------------------------------------
# TisTpgDRM-Management ::= SEQUENCE {
#  generationTime	TimestampIts,
#  vehicleType UNVehicleClassifcation,
#  costumerContract CustomerContract OPTIONAL,
#  tisProfile	TisProfile, 
#  ... 
# }
# ------------------------------------------------------------------------------

TimestampIts generationTime

UNVehicleClassifcation vehicleType

# optional: True
CustomerContract costumerContract

TisProfile tisProfile
