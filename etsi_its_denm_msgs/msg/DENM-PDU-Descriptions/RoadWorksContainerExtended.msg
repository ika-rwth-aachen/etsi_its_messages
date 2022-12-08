# ------------------------------------------------------------------------------
# RoadWorksContainerExtended ::= SEQUENCE {
# 	lightBarSirenInUse LightBarSirenInUse OPTIONAL,
# 	closedLanes ClosedLanes OPTIONAL,
# 	restriction RestrictedTypes OPTIONAL,
# 	speedLimit SpeedLimit OPTIONAL,
# 	incidentIndication CauseCode OPTIONAL,
# 	recommendedPath ItineraryPath OPTIONAL,
# 	startingPointSpeedLimit DeltaReferencePosition OPTIONAL,
# 	trafficFlowRule TrafficRule OPTIONAL,
# 	referenceDenms ReferenceDenms OPTIONAL
#  }
# ------------------------------------------------------------------------------

LightBarSirenInUse lightBarSirenInUse
bool lightBarSirenInUse_isPresent

ClosedLanes closedLanes
bool closedLanes_isPresent

RestrictedTypes restriction
bool restriction_isPresent

SpeedLimit speedLimit
bool speedLimit_isPresent

CauseCode incidentIndication
bool incidentIndication_isPresent

ItineraryPath recommendedPath
bool recommendedPath_isPresent

DeltaReferencePosition startingPointSpeedLimit
bool startingPointSpeedLimit_isPresent

TrafficRule trafficFlowRule
bool trafficFlowRule_isPresent

ReferenceDenms referenceDenms
bool referenceDenms_isPresent
