# ------------------------------------------------------------------------------
# AdverseWeatherCondition-ExtremeWeatherConditionSubCauseCode ::= INTEGER {unavailable(0), strongWinds(1), damagingHail(2), hurricane(3), thunderstorm(4), tornado(5), blizzard(6)} (0..255)
# ------------------------------------------------------------------------------

# restricted-to: [(0, 255)]
uint8 UNAVAILABLE = 0
uint8 STRONG_WINDS = 1
uint8 DAMAGING_HAIL = 2
uint8 HURRICANE = 3
uint8 THUNDERSTORM = 4
uint8 TORNADO = 5
uint8 BLIZZARD = 6
uint8 value
