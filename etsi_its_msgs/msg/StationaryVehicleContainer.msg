int32 stationarySince
int32 STATIONARY_SINCE_LESS_THAN_1_MINUTE = 0
int32 STATIONARY_SINCE_LESS_THAN_2_MINUTES = 1
int32 STATIONARY_SINCE_LESS_THAN_1_5_MINUTES = 2
int32 STATIONARY_SINCE_EQUAL_OR_GREATER_1_5_MINUTES = 3

# optional: True
CauseCode stationaryCause

# optional: True
DangerousGoodsExtended carryingDangerousGoods

# restricted-to: [(0, 127)]
int32 numberOfOccupants
int32 NUMBER_OF_OCCUPANTS_ONE_OCCUPANT = 1
int32 NUMBER_OF_OCCUPANTS_UNAVAILABLE = 127

# optional: True
VehicleIdentification vehicleIdentification

# named-bits: [('hydrogenStorage', '0'), ('electricEnergyStorage', '1'), ('liquidPropaneGas', '2'), ('compressedNaturalGas', '3'), ('diesel', '4'), ('gasoline', '5'), ('ammonia', '6')]
# size: [7]
string energyStorageType

