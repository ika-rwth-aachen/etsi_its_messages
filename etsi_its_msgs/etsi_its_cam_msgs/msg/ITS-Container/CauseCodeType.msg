# ------------------------------------------------------------------------------
# CauseCodeType ::= INTEGER {
#     reserved (0),
#     trafficCondition (1),
#     accident (2),
#     roadworks (3),
#         impassability (5),
#     adverseWeatherCondition-Adhesion (6),
#         aquaplannning (7),
#     hazardousLocation-SurfaceCondition (9),
#     hazardousLocation-ObstacleOnTheRoad (10),
#     hazardousLocation-AnimalOnTheRoad (11),
#     humanPresenceOnTheRoad (12),
#     wrongWayDriving (14),
#     rescueAndRecoveryWorkInProgress (15),
#     adverseWeatherCondition-ExtremeWeatherCondition (17),
#     adverseWeatherCondition-Visibility (18),
#     adverseWeatherCondition-Precipitation (19),
#     slowVehicle (26),
#     dangerousEndOfQueue (27),
#     vehicleBreakdown (91),
#     postCrash (92),
#     humanProblem (93),
#     stationaryVehicle (94),
#     emergencyVehicleApproaching (95),
#     hazardousLocation-DangerousCurve (96),
#     collisionRisk (97),
#     signalViolation (98),
#     dangerousSituation (99)
# } (0..255)
# ------------------------------------------------------------------------------

# restricted-to: [(0, 255)]
int64 RESERVED = 0
int64 TRAFFIC_CONDITION = 1
int64 ACCIDENT = 2
int64 ROADWORKS = 3
int64 IMPASSABILITY = 5
int64 ADVERSE_WEATHER_CONDITION_ADHESION = 6
int64 AQUAPLANNNING = 7
int64 HAZARDOUS_LOCATION_SURFACE_CONDITION = 9
int64 HAZARDOUS_LOCATION_OBSTACLE_ON_THE_ROAD = 10
int64 HAZARDOUS_LOCATION_ANIMAL_ON_THE_ROAD = 11
int64 HUMAN_PRESENCE_ON_THE_ROAD = 12
int64 WRONG_WAY_DRIVING = 14
int64 RESCUE_AND_RECOVERY_WORK_IN_PROGRESS = 15
int64 ADVERSE_WEATHER_CONDITION_EXTREME_WEATHER_CONDITION = 17
int64 ADVERSE_WEATHER_CONDITION_VISIBILITY = 18
int64 ADVERSE_WEATHER_CONDITION_PRECIPITATION = 19
int64 SLOW_VEHICLE = 26
int64 DANGEROUS_END_OF_QUEUE = 27
int64 VEHICLE_BREAKDOWN = 91
int64 POST_CRASH = 92
int64 HUMAN_PROBLEM = 93
int64 STATIONARY_VEHICLE = 94
int64 EMERGENCY_VEHICLE_APPROACHING = 95
int64 HAZARDOUS_LOCATION_DANGEROUS_CURVE = 96
int64 COLLISION_RISK = 97
int64 SIGNAL_VIOLATION = 98
int64 DANGEROUS_SITUATION = 99
int64 value
