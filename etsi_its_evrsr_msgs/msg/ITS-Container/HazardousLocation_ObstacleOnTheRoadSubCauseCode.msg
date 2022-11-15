# ------------------------------------------------------------------------------
# HazardousLocation-ObstacleOnTheRoadSubCauseCode ::= INTEGER {unavailable(0), shedLoad(1), partsOfVehicles(2), partsOfTyres(3), bigObjects(4), fallenTrees(5), hubCaps(6), waitingVehicles(7)} (0..255)
# ------------------------------------------------------------------------------

# restricted-to: [(0, 255)]
uint8 UNAVAILABLE = 0
uint8 SHED_LOAD = 1
uint8 PARTS_OF_VEHICLES = 2
uint8 PARTS_OF_TYRES = 3
uint8 BIG_OBJECTS = 4
uint8 FALLEN_TREES = 5
uint8 HUB_CAPS = 6
uint8 WAITING_VEHICLES = 7
uint8 value
