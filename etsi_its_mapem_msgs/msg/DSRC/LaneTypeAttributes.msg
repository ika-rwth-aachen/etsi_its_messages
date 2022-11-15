# ------------------------------------------------------------------------------
# LaneTypeAttributes ::= CHOICE {
#   vehicle        LaneAttributes-Vehicle,
#   crosswalk      LaneAttributes-Crosswalk,
#   bikeLane       LaneAttributes-Bike,
#   sidewalk       LaneAttributes-Sidewalk,
#   median         LaneAttributes-Barrier,
#   striping       LaneAttributes-Striping,
#   trackedVehicle LaneAttributes-TrackedVehicle,
#   parking        LaneAttributes-Parking,
#   ...
# }
# ------------------------------------------------------------------------------

uint8 choice

LaneAttributes_Vehicle vehicle
uint8 CHOICE_VEHICLE = 0
LaneAttributes_Crosswalk crosswalk
uint8 CHOICE_CROSSWALK = 1
LaneAttributes_Bike bikeLane
uint8 CHOICE_BIKE_LANE = 2
LaneAttributes_Sidewalk sidewalk
uint8 CHOICE_SIDEWALK = 3
LaneAttributes_Barrier median
uint8 CHOICE_MEDIAN = 4
LaneAttributes_Striping striping
uint8 CHOICE_STRIPING = 5
LaneAttributes_TrackedVehicle trackedVehicle
uint8 CHOICE_TRACKED_VEHICLE = 6
LaneAttributes_Parking parking
uint8 CHOICE_PARKING = 7
