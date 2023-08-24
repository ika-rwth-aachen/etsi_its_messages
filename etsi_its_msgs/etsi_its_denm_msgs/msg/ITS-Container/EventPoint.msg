# ------------------------------------------------------------------------------
# EventPoint ::= SEQUENCE {
#     eventPosition DeltaReferencePosition,
#     eventDeltaTime PathDeltaTime OPTIONAL,
#     informationQuality InformationQuality
# }
# ------------------------------------------------------------------------------

DeltaReferencePosition event_position

PathDeltaTime event_delta_time
bool event_delta_time_is_present

InformationQuality information_quality

