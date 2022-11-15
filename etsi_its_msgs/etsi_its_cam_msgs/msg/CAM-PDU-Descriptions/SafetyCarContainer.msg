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

# optional: True
TrafficRule trafficRule

# optional: True
SpeedLimit speedLimit
