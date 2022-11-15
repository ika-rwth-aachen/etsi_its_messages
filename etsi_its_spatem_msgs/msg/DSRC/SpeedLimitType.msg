# ------------------------------------------------------------------------------
# SpeedLimitType ::= ENUMERATED {
#    unknown,
#    maxSpeedInSchoolZone,
#    maxSpeedInSchoolZoneWhenChildrenArePresent,
#    maxSpeedInConstructionZone,
#    vehicleMinSpeed,
#    vehicleMaxSpeed,
#    vehicleNightMaxSpeed,
#    truckMinSpeed,
#    truckMaxSpeed,
#    truckNightMaxSpeed,
#    vehiclesWithTrailersMinSpeed,
#    vehiclesWithTrailersMaxSpeed,
#    vehiclesWithTrailersNightMaxSpeed,
#    ...
# }
# ------------------------------------------------------------------------------

uint8 UNKNOWN = 0
uint8 MAX_SPEED_IN_SCHOOL_ZONE = 1
uint8 MAX_SPEED_IN_SCHOOL_ZONE_WHEN_CHILDREN_ARE_PRESENT = 2
uint8 MAX_SPEED_IN_CONSTRUCTION_ZONE = 3
uint8 VEHICLE_MIN_SPEED = 4
uint8 VEHICLE_MAX_SPEED = 5
uint8 VEHICLE_NIGHT_MAX_SPEED = 6
uint8 TRUCK_MIN_SPEED = 7
uint8 TRUCK_MAX_SPEED = 8
uint8 TRUCK_NIGHT_MAX_SPEED = 9
uint8 VEHICLES_WITH_TRAILERS_MIN_SPEED = 10
uint8 VEHICLES_WITH_TRAILERS_MAX_SPEED = 11
uint8 VEHICLES_WITH_TRAILERS_NIGHT_MAX_SPEED = 12
uint8 value
