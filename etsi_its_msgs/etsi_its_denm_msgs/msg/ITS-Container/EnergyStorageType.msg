# ------------------------------------------------------------------------------
# EnergyStorageType ::= BIT STRING {hydrogenStorage(0), electricEnergyStorage(1), liquidPropaneGas(2), compressedNaturalGas(3), diesel(4), gasoline(5), ammonia(6)} (SIZE(7))
# ------------------------------------------------------------------------------

uint8[] value
uint8 bits_unused
uint8 SIZE_BITS = 7
uint8 BIT_INDEX_HYDROGEN_STORAGE = 0
uint8 BIT_INDEX_ELECTRIC_ENERGY_STORAGE = 1
uint8 BIT_INDEX_LIQUID_PROPANE_GAS = 2
uint8 BIT_INDEX_COMPRESSED_NATURAL_GAS = 3
uint8 BIT_INDEX_DIESEL = 4
uint8 BIT_INDEX_GASOLINE = 5
uint8 BIT_INDEX_AMMONIA = 6

