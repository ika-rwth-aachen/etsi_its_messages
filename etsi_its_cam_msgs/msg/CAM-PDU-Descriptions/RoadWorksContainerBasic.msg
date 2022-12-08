# ------------------------------------------------------------------------------
#  RoadWorksContainerBasic ::= SEQUENCE {
#   roadworksSubCauseCode RoadworksSubCauseCode OPTIONAL,
#   lightBarSirenInUse LightBarSirenInUse,
#   closedLanes ClosedLanes OPTIONAL
#  }
# ------------------------------------------------------------------------------

# optional: True
RoadworksSubCauseCode roadworksSubCauseCode
bool roadworksSubCauseCode_isPresent

LightBarSirenInUse lightBarSirenInUse

# optional: True
ClosedLanes closedLanes
bool closedLanes_isPresent
