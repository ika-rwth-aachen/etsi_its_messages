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

# optional: True
LanePosition lanePosition

# optional: True
ImpactReductionContainer impactReduction

# optional: True
Temperature externalTemperature

# optional: True
RoadWorksContainerExtended roadWorks

# optional: True
PositioningSolutionType positioningSolution

# optional: True
StationaryVehicleContainer stationaryVehicle
