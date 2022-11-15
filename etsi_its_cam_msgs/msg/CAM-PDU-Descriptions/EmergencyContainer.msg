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

# optional: True
EmergencyPriority emergencyPriority
