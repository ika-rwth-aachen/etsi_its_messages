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

ActionID action_id

TimestampIts detection_time

TimestampIts reference_time

Termination termination
bool termination_is_present

ReferencePosition event_position

RelevanceDistance relevance_distance
bool relevance_distance_is_present

RelevanceTrafficDirection relevance_traffic_direction
bool relevance_traffic_direction_is_present

ValidityDuration validity_duration
uint16 DEFAULT_VALIDITY_DURATION = 600

TransmissionInterval transmission_interval
bool transmission_interval_is_present

StationType station_type

