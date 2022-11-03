# ------------------------------------------------------------------------------
# TyrePressure ::= INTEGER { invalid (0), lessThanOneBar (1), oneBar (2), oneBarPlusTwoAndHalfKPa (3), inflation (254), overflow(255)  } (0..255)
# ------------------------------------------------------------------------------

# restricted-to: [(0, 255)]
uint8 INVALID = 0
uint8 LESS_THAN_ONE_BAR = 1
uint8 ONE_BAR = 2
uint8 ONE_BAR_PLUS_TWO_AND_HALF_K_PA = 3
uint8 INFLATION = 254
uint8 OVERFLOW = 255
uint8 value
