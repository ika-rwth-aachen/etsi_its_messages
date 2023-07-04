# ------------------------------------------------------------------------------
# RoadworksSubCauseCode ::= INTEGER {unavailable(0), majorRoadworks(1), roadMarkingWork(2), slowMovingRoadMaintenance(3), shortTermStationaryRoadworks(4), streetCleaning(5), winterService(6)} (0..255)
# ------------------------------------------------------------------------------

uint8 value
uint8 MIN = 0
uint8 MAX = 255
uint8 UNAVAILABLE = 0
uint8 MAJOR_ROADWORKS = 1
uint8 ROAD_MARKING_WORK = 2
uint8 SLOW_MOVING_ROAD_MAINTENANCE = 3
uint8 SHORT_TERM_STATIONARY_ROADWORKS = 4
uint8 STREET_CLEANING = 5
uint8 WINTER_SERVICE = 6

