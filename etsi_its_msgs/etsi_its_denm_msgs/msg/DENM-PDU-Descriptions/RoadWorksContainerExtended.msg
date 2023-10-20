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

LightBarSirenInUse light_bar_siren_in_use
bool light_bar_siren_in_use_is_present

ClosedLanes closed_lanes
bool closed_lanes_is_present

RestrictedTypes restriction
bool restriction_is_present

SpeedLimit speed_limit
bool speed_limit_is_present

CauseCode incident_indication
bool incident_indication_is_present

ItineraryPath recommended_path
bool recommended_path_is_present

DeltaReferencePosition starting_point_speed_limit
bool starting_point_speed_limit_is_present

TrafficRule traffic_flow_rule
bool traffic_flow_rule_is_present

ReferenceDenms reference_denms
bool reference_denms_is_present

