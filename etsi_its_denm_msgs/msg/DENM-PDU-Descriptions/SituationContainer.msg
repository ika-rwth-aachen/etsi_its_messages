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

# optional: True
CauseCode linkedCause

# optional: True
EventHistory eventHistory
