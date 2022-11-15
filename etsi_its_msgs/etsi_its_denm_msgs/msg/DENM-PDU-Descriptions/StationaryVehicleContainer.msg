# ------------------------------------------------------------------------------
# StationaryVehicleContainer ::= SEQUENCE {
# 	stationarySince StationarySince OPTIONAL,
# 	stationaryCause CauseCode OPTIONAL,
# 	carryingDangerousGoods DangerousGoodsExtended OPTIONAL,
# 	numberOfOccupants NumberOfOccupants OPTIONAL,
# 	vehicleIdentification VehicleIdentification OPTIONAL,
# 	energyStorageType EnergyStorageType OPTIONAL
# }
# ------------------------------------------------------------------------------

# optional: True
StationarySince stationarySince

# optional: True
CauseCode stationaryCause

# optional: True
DangerousGoodsExtended carryingDangerousGoods

# optional: True
NumberOfOccupants numberOfOccupants

# optional: True
VehicleIdentification vehicleIdentification

# optional: True
EnergyStorageType energyStorageType
