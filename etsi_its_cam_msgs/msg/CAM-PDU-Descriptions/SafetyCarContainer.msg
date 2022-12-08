# ------------------------------------------------------------------------------
# SafetyCarContainer ::= SEQUENCE {
# 	lightBarSirenInUse LightBarSirenInUse,
# 	incidentIndication CauseCode OPTIONAL,
# 	trafficRule TrafficRule OPTIONAL,
# 	speedLimit SpeedLimit OPTIONAL
# }
# ------------------------------------------------------------------------------

LightBarSirenInUse lightBarSirenInUse

# optional: True
CauseCode incidentIndication
bool incidentIndication_isPresent

# optional: True
TrafficRule trafficRule
bool trafficRule_isPresent

# optional: True
SpeedLimit speedLimit
bool speedLimit_isPresent
