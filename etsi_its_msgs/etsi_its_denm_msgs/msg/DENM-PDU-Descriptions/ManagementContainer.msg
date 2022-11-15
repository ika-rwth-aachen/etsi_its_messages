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

# optional: True
Termination termination

ReferencePosition eventPosition

# optional: True
RelevanceDistance relevanceDistance

# optional: True
RelevanceTrafficDirection relevanceTrafficDirection

# default: defaultValidity
ValidityDuration validityDuration

# optional: True
TransmissionInterval transmissionInterval

StationType stationType
