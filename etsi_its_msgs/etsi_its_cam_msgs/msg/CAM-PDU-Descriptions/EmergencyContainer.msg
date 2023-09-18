# ------------------------------------------------------------------------------
# EmergencyContainer ::= SEQUENCE {
# 	lightBarSirenInUse LightBarSirenInUse,
# 	incidentIndication CauseCode OPTIONAL,
# 	emergencyPriority EmergencyPriority OPTIONAL
# }
# ------------------------------------------------------------------------------

LightBarSirenInUse light_bar_siren_in_use

CauseCode incident_indication
bool incident_indication_is_present

EmergencyPriority emergency_priority
bool emergency_priority_is_present

