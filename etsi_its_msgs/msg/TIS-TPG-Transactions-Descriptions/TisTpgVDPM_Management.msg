# ------------------------------------------------------------------------------
# TisTpgVDPM-Management ::= SEQUENCE {
#  generationTime	TimestampIts,
#  tisProfile	TisProfile	OPTIONAL,
#  language Language,
#  vehicleType UNVehicleClassifcation,
#  tyreTempCondition TyreTempCondition,
#  fillingStatus FillingStatus,
#  pairingID PairingID OPTIONAL,
#  ...
# }
# ------------------------------------------------------------------------------

TimestampIts generationTime

# optional: True
TisProfile tisProfile

Language language

UNVehicleClassifcation vehicleType

TyreTempCondition tyreTempCondition

FillingStatus fillingStatus

# optional: True
PairingID pairingID
