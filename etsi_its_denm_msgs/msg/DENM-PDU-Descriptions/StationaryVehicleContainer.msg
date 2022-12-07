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

StationarySince stationarySince
bool stationarySince_isPresent

CauseCode stationaryCause
bool stationaryCause_isPresent

DangerousGoodsExtended carryingDangerousGoods
bool carryingDangerousGoods_isPresent

NumberOfOccupants numberOfOccupants
bool numberOfOccupants_isPresent

VehicleIdentification vehicleIdentification
bool vehicleIdentification_isPresent

EnergyStorageType energyStorageType
bool energyStorageType_isPresent
