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

# optional: True
ZoneLength queueLength

# optional: True
ZoneLength availableStorageLength

# optional: True
WaitOnStopline waitOnStop

# optional: True
PedestrianBicycleDetect pedBicycleDetect

# size: [(1, 4)]
# optional: True
RegionalExtension[] array
