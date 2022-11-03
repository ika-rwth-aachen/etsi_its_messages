# ------------------------------------------------------------------------------
# LaneAttributes-Crosswalk ::= BIT STRING { 
#   crosswalkRevocableLane  (0),
#   bicyleUseAllowed        (1),
#   isXwalkFlyOverLane      (2),
#   fixedCycleTime          (3),
#   biDirectionalCycleTimes (4),
#   hasPushToWalkButton     (5),
#   audioSupport            (6),
#   rfSignalRequestPresent  (7),
#   unsignalizedSegmentsPresent  (8)
# } (SIZE (16))
# ------------------------------------------------------------------------------

# named-bits: [('crosswalkRevocableLane', '0'), ('bicyleUseAllowed', '1'), ('isXwalkFlyOverLane', '2'), ('fixedCycleTime', '3'), ('biDirectionalCycleTimes', '4'), ('hasPushToWalkButton', '5'), ('audioSupport', '6'), ('rfSignalRequestPresent', '7'), ('unsignalizedSegmentsPresent', '8')]
# size: [16]
string value
