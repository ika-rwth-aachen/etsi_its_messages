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

StationarySince stationary_since
bool stationary_since_is_present

CauseCode stationary_cause
bool stationary_cause_is_present

DangerousGoodsExtended carrying_dangerous_goods
bool carrying_dangerous_goods_is_present

NumberOfOccupants number_of_occupants
bool number_of_occupants_is_present

VehicleIdentification vehicle_identification
bool vehicle_identification_is_present

EnergyStorageType energy_storage_type
bool energy_storage_type_is_present

