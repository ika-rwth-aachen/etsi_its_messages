# ------------------------------------------------------------------------------
# ItsStationPosition ::=	SEQUENCE {
#   stationID      StationID,
#   laneID         LaneID             OPTIONAL,
#   nodeXY         NodeOffsetPointXY  OPTIONAL,
#   timeReference  TimeReference      OPTIONAL,
#   ...
# }
# ------------------------------------------------------------------------------

StationID stationID

# optional: True
LaneID laneID

# optional: True
NodeOffsetPointXY nodeXY

# optional: True
TimeReference timeReference
