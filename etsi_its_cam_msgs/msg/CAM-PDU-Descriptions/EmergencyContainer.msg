# ------------------------------------------------------------------------------
# EmergencyContainer ::= SEQUENCE {
# 	lightBarSirenInUse LightBarSirenInUse,
# 	incidentIndication CauseCode OPTIONAL,
# 	emergencyPriority EmergencyPriority OPTIONAL
# }
# ------------------------------------------------------------------------------

LightBarSirenInUse lightBarSirenInUse

CauseCode incidentIndication
bool incidentIndication_isPresent

EmergencyPriority emergencyPriority
bool emergencyPriority_isPresent
