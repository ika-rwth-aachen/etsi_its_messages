# ------------------------------------------------------------------------------
# SituationContainer ::= SEQUENCE {
# 	informationQuality InformationQuality,
# 	eventType CauseCode,
# 	linkedCause CauseCode OPTIONAL,
# 	eventHistory EventHistory OPTIONAL,
# 	...
# }
# ------------------------------------------------------------------------------

InformationQuality information_quality

CauseCode event_type

CauseCode linked_cause
bool linked_cause_is_present

EventHistory event_history
bool event_history_is_present

