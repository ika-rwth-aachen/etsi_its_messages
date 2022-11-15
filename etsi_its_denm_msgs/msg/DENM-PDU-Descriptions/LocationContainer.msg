# ------------------------------------------------------------------------------
# LocationContainer ::= SEQUENCE {
# 	eventSpeed Speed OPTIONAL,
# 	eventPositionHeading Heading OPTIONAL,
# 	traces Traces,
# 	roadType RoadType OPTIONAL,
# 	...
# }
# ------------------------------------------------------------------------------

# optional: True
Speed eventSpeed

# optional: True
Heading eventPositionHeading

Traces traces

# optional: True
RoadType roadType
