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

LanePosition lanePosition
bool lanePosition_isPresent

ImpactReductionContainer impactReduction
bool impactReduction_isPresent

Temperature externalTemperature
bool externalTemperature_isPresent

RoadWorksContainerExtended roadWorks
bool roadWorks_isPresent

PositioningSolutionType positioningSolution
bool positioningSolution_isPresent

StationaryVehicleContainer stationaryVehicle
bool stationaryVehicle_isPresent
