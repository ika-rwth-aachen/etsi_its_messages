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

# optional: True
LightBarSirenInUse lightBarSirenInUse

# optional: True
ClosedLanes closedLanes

# optional: True
RestrictedTypes restriction

# optional: True
SpeedLimit speedLimit

# optional: True
CauseCode incidentIndication

# optional: True
ItineraryPath recommendedPath

# optional: True
DeltaReferencePosition startingPointSpeedLimit

# optional: True
TrafficRule trafficFlowRule

# optional: True
ReferenceDenms referenceDenms
