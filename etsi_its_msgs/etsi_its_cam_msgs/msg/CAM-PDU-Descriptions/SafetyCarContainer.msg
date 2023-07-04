# ------------------------------------------------------------------------------
# SafetyCarContainer ::= SEQUENCE {
# 	lightBarSirenInUse LightBarSirenInUse,
# 	incidentIndication CauseCode OPTIONAL,
# 	trafficRule TrafficRule OPTIONAL,
# 	speedLimit SpeedLimit OPTIONAL
# }
# ------------------------------------------------------------------------------

LightBarSirenInUse lightBarSirenInUse

CauseCode incidentIndication
bool incidentIndication_isPresent

TrafficRule trafficRule
bool trafficRule_isPresent

SpeedLimit speedLimit
bool speedLimit_isPresent

