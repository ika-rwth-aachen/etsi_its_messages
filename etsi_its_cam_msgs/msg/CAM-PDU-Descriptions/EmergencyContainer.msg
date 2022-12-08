# ------------------------------------------------------------------------------
# EmergencyContainer ::= SEQUENCE {
# 	lightBarSirenInUse LightBarSirenInUse,
# 	incidentIndication CauseCode OPTIONAL,
# 	emergencyPriority EmergencyPriority OPTIONAL
# }
# ------------------------------------------------------------------------------

LightBarSirenInUse lightBarSirenInUse

# optional: True
CauseCode incidentIndication
bool incidentIndication_isPresent

# optional: True
EmergencyPriority emergencyPriority
bool emergencyPriority_isPresent
