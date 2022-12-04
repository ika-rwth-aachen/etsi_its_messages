# ------------------------------------------------------------------------------
# RoadworksSubCauseCode ::= INTEGER {unavailable(0), majorRoadworks(1), roadMarkingWork(2), slowMovingRoadMaintenance(3), shortTermStationaryRoadworks(4), streetCleaning(5), winterService(6)} (0..255)
# ------------------------------------------------------------------------------

# restricted-to: [(0, 255)]
int64 UNAVAILABLE = 0
int64 MAJOR_ROADWORKS = 1
int64 ROAD_MARKING_WORK = 2
int64 SLOW_MOVING_ROAD_MAINTENANCE = 3
int64 SHORT_TERM_STATIONARY_ROADWORKS = 4
int64 STREET_CLEANING = 5
int64 WINTER_SERVICE = 6
int64 value
