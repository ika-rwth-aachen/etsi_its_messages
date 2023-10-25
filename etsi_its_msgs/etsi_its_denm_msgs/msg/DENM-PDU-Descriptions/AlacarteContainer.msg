# ------------------------------------------------------------------------------
# AlacarteContainer ::= SEQUENCE {
# 	lanePosition LanePosition OPTIONAL,
# 	impactReduction ImpactReductionContainer OPTIONAL,
# 	externalTemperature Temperature OPTIONAL,
# 	roadWorks RoadWorksContainerExtended OPTIONAL,
# 	positioningSolution PositioningSolutionType OPTIONAL,
# 	stationaryVehicle StationaryVehicleContainer OPTIONAL,
# 	...
# }
# ------------------------------------------------------------------------------

LanePosition lane_position
bool lane_position_is_present

ImpactReductionContainer impact_reduction
bool impact_reduction_is_present

Temperature external_temperature
bool external_temperature_is_present

RoadWorksContainerExtended road_works
bool road_works_is_present

PositioningSolutionType positioning_solution
bool positioning_solution_is_present

StationaryVehicleContainer stationary_vehicle
bool stationary_vehicle_is_present

