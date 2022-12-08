# ------------------------------------------------------------------------------
# SituationContainer ::= SEQUENCE {
# 	informationQuality InformationQuality,
# 	eventType CauseCode,
# 	linkedCause CauseCode OPTIONAL,
# 	eventHistory EventHistory OPTIONAL,
# 	...
# }
# ------------------------------------------------------------------------------

InformationQuality informationQuality

CauseCode eventType

CauseCode linkedCause
bool linkedCause_isPresent

EventHistory eventHistory
bool eventHistory_isPresent
