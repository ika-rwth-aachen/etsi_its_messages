# ------------------------------------------------------------------------------
# PublicTransportContainer ::= SEQUENCE {
# 	embarkationStatus EmbarkationStatus,
# 	ptActivation PtActivation OPTIONAL
# }
# ------------------------------------------------------------------------------

EmbarkationStatus embarkationStatus

# optional: True
PtActivation ptActivation
bool ptActivation_isPresent
