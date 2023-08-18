# ------------------------------------------------------------------------------
# SafetyCarContainer ::= SEQUENCE {
# 	lightBarSirenInUse LightBarSirenInUse,
# 	incidentIndication CauseCode OPTIONAL,
# 	trafficRule TrafficRule OPTIONAL,
# 	speedLimit SpeedLimit OPTIONAL
# }
# ------------------------------------------------------------------------------

LightBarSirenInUse light_bar_siren_in_use

CauseCode incident_indication
bool incident_indication_is_present

TrafficRule traffic_rule
bool traffic_rule_is_present

SpeedLimit speed_limit
bool speed_limit_is_present

