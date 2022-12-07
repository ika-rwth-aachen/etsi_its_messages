# ------------------------------------------------------------------------------
# ConnectionManeuverAssist ::= SEQUENCE {
#   connectionID         LaneConnectionID,
#   queueLength          ZoneLength OPTIONAL,
#   availableStorageLength ZoneLength OPTIONAL,
#   waitOnStop           WaitOnStopline OPTIONAL,
#   pedBicycleDetect     PedestrianBicycleDetect OPTIONAL,
#   regional  SEQUENCE (SIZE(1..4)) OF 
#             RegionalExtension {{Reg-ConnectionManeuverAssist}} OPTIONAL,
# ------------------------------------------------------------------------------

LaneConnectionID connectionID

ZoneLength queueLength
bool queueLength_isPresent

ZoneLength availableStorageLength
bool availableStorageLength_isPresent

WaitOnStopline waitOnStop
bool waitOnStop_isPresent

PedestrianBicycleDetect pedBicycleDetect
bool pedBicycleDetect_isPresent

# size: [(1, 4)]
RegionalExtension[] array
bool regional_isPresent
