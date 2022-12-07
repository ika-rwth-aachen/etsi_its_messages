# ------------------------------------------------------------------------------
# ManagementContainer ::= SEQUENCE {
# 	actionID ActionID,
# 	detectionTime TimestampIts,
# 	referenceTime TimestampIts,
# 	termination Termination OPTIONAL,
# 	eventPosition ReferencePosition,
# 	relevanceDistance RelevanceDistance OPTIONAL,
# 	relevanceTrafficDirection RelevanceTrafficDirection OPTIONAL,
# 	validityDuration ValidityDuration DEFAULT defaultValidity,
# 	transmissionInterval TransmissionInterval OPTIONAL,
# 	stationType	StationType,
# 	...
# }
# ------------------------------------------------------------------------------

ActionID actionID

TimestampIts detectionTime

TimestampIts referenceTime

Termination termination
bool termination_isPresent

ReferencePosition eventPosition

RelevanceDistance relevanceDistance
bool relevanceDistance_isPresent

RelevanceTrafficDirection relevanceTrafficDirection
bool relevanceTrafficDirection_isPresent

# default: defaultValidity
ValidityDuration validityDuration

TransmissionInterval transmissionInterval
bool transmissionInterval_isPresent

StationType stationType
