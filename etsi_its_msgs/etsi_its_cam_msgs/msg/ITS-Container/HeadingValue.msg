# ------------------------------------------------------------------------------
# HeadingValue ::= INTEGER {wgs84North(0), wgs84East(900), wgs84South(1800), wgs84West(2700), unavailable(3601)} (0..3601)
# ------------------------------------------------------------------------------

uint16 value
uint16 MIN = 0
uint16 MAX = 3601
uint16 WGS_8_4_NORTH = 0
uint16 WGS_8_4_EAST = 900
uint16 WGS_8_4_SOUTH = 1800
uint16 WGS_8_4_WEST = 2700
uint16 UNAVAILABLE = 3601

