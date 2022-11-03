# ------------------------------------------------------------------------------
# TyreAirTemperature ::= INTEGER { invalid (0), lessThanMinus50Celsius(1), minus50Celsius(2), minus49Celsius(3), zeroCelsius(52), overflowThreshold1(240), overflowThreshold2(241), overflowThreshold3(242), overflowThreshold4(243), overflowThreshold5(244), overflowThreshold6(245), overflowThreshold7(246), overflowThreshold8(247), overflowThreshold9(248), overflowThreshold10(249), overflowThreshold11(250), overflowThreshold12(251), overflowThreshold13(252), overflowThreshold14(253), overflowThreshold15(254), overflowThreshold16(255) } (0..255)
# ------------------------------------------------------------------------------

# restricted-to: [(0, 255)]
uint8 INVALID = 0
uint8 LESS_THAN_MINUS_5_0_CELSIUS = 1
uint8 MINUS_5_0_CELSIUS = 2
uint8 MINUS_4_9_CELSIUS = 3
uint8 ZERO_CELSIUS = 52
uint8 OVERFLOW_THRESHOLD_1 = 240
uint8 OVERFLOW_THRESHOLD_2 = 241
uint8 OVERFLOW_THRESHOLD_3 = 242
uint8 OVERFLOW_THRESHOLD_4 = 243
uint8 OVERFLOW_THRESHOLD_5 = 244
uint8 OVERFLOW_THRESHOLD_6 = 245
uint8 OVERFLOW_THRESHOLD_7 = 246
uint8 OVERFLOW_THRESHOLD_8 = 247
uint8 OVERFLOW_THRESHOLD_9 = 248
uint8 OVERFLOW_THRESHOLD_1_0 = 249
uint8 OVERFLOW_THRESHOLD_1_1 = 250
uint8 OVERFLOW_THRESHOLD_1_2 = 251
uint8 OVERFLOW_THRESHOLD_1_3 = 252
uint8 OVERFLOW_THRESHOLD_1_4 = 253
uint8 OVERFLOW_THRESHOLD_1_5 = 254
uint8 OVERFLOW_THRESHOLD_1_6 = 255
uint8 value
