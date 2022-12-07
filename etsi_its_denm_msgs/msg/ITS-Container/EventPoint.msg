# ------------------------------------------------------------------------------
# EventPoint ::= SEQUENCE {
#     eventPosition DeltaReferencePosition,
#     eventDeltaTime PathDeltaTime OPTIONAL,
#     informationQuality InformationQuality
# }
# ------------------------------------------------------------------------------

DeltaReferencePosition eventPosition

PathDeltaTime eventDeltaTime
bool eventDeltaTime_isPresent

InformationQuality informationQuality
