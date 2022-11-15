# ------------------------------------------------------------------------------
# HazardousLocation-SurfaceConditionSubCauseCode ::= INTEGER {unavailable(0), rockfalls(1), earthquakeDamage(2), sewerCollapse(3), subsidence(4), snowDrifts(5), stormDamage(6), burstPipe(7), volcanoEruption(8), fallingIce(9)} (0..255)
# ------------------------------------------------------------------------------

# restricted-to: [(0, 255)]
uint8 UNAVAILABLE = 0
uint8 ROCKFALLS = 1
uint8 EARTHQUAKE_DAMAGE = 2
uint8 SEWER_COLLAPSE = 3
uint8 SUBSIDENCE = 4
uint8 SNOW_DRIFTS = 5
uint8 STORM_DAMAGE = 6
uint8 BURST_PIPE = 7
uint8 VOLCANO_ERUPTION = 8
uint8 FALLING_ICE = 9
uint8 value
