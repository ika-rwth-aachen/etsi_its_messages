# ------------------------------------------------------------------------------
# SpecialTransportType ::= BIT STRING {heavyLoad(0), excessWidth(1), excessLength(2), excessHeight(3)} (SIZE(4))
# ------------------------------------------------------------------------------

bool[4] value
uint8 INDEX_HEAVY_LOAD = 0
uint8 INDEX_EXCESS_WIDTH = 1
uint8 INDEX_EXCESS_LENGTH = 2
uint8 INDEX_EXCESS_HEIGHT = 3

