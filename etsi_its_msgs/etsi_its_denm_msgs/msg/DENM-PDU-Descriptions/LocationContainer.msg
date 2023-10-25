# ------------------------------------------------------------------------------
# LocationContainer ::= SEQUENCE {
# 	eventSpeed Speed OPTIONAL,
# 	eventPositionHeading Heading OPTIONAL,
# 	traces Traces,
# 	roadType RoadType OPTIONAL,
# 	...
# }
# ------------------------------------------------------------------------------

Speed event_speed
bool event_speed_is_present

Heading event_position_heading
bool event_position_heading_is_present

Traces traces

RoadType road_type
bool road_type_is_present

