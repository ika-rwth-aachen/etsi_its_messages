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

uint8 value
uint8 MIN = 0
uint8 MAX = 255
uint8 RESERVED = 0
uint8 TRAFFIC_CONDITION = 1
uint8 ACCIDENT = 2
uint8 ROADWORKS = 3
uint8 IMPASSABILITY = 5
uint8 ADVERSE_WEATHER_CONDITION_ADHESION = 6
uint8 AQUAPLANNNING = 7
uint8 HAZARDOUS_LOCATION_SURFACE_CONDITION = 9
uint8 HAZARDOUS_LOCATION_OBSTACLE_ON_THE_ROAD = 10
uint8 HAZARDOUS_LOCATION_ANIMAL_ON_THE_ROAD = 11
uint8 HUMAN_PRESENCE_ON_THE_ROAD = 12
uint8 WRONG_WAY_DRIVING = 14
uint8 RESCUE_AND_RECOVERY_WORK_IN_PROGRESS = 15
uint8 ADVERSE_WEATHER_CONDITION_EXTREME_WEATHER_CONDITION = 17
uint8 ADVERSE_WEATHER_CONDITION_VISIBILITY = 18
uint8 ADVERSE_WEATHER_CONDITION_PRECIPITATION = 19
uint8 SLOW_VEHICLE = 26
uint8 DANGEROUS_END_OF_QUEUE = 27
uint8 VEHICLE_BREAKDOWN = 91
uint8 POST_CRASH = 92
uint8 HUMAN_PROBLEM = 93
uint8 STATIONARY_VEHICLE = 94
uint8 EMERGENCY_VEHICLE_APPROACHING = 95
uint8 HAZARDOUS_LOCATION_DANGEROUS_CURVE = 96
uint8 COLLISION_RISK = 97
uint8 SIGNAL_VIOLATION = 98
uint8 DANGEROUS_SITUATION = 99

