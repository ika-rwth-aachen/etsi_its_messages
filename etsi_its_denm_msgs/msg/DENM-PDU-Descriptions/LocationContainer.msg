# ------------------------------------------------------------------------------
# LocationContainer ::= SEQUENCE {
# 	eventSpeed Speed OPTIONAL,
# 	eventPositionHeading Heading OPTIONAL,
# 	traces Traces,
# 	roadType RoadType OPTIONAL,
# 	...
# }
# ------------------------------------------------------------------------------

Speed eventSpeed
bool eventSpeed_isPresent

Heading eventPositionHeading
bool eventPositionHeading_isPresent

Traces traces

RoadType roadType
bool roadType_isPresent
