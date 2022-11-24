# ------------------------------------------------------------------------------
# LaneAttributes-Bike ::= BIT STRING {
#   bikeRevocableLane       (0),
#   pedestrianUseAllowed    (1),
#   isBikeFlyOverLane       (2),
#   fixedCycleTime          (3),
#   biDirectionalCycleTimes (4),
#   isolatedByBarrier       (5),
#   unsignalizedSegmentsPresent  (6)
# } (SIZE (16))
# ------------------------------------------------------------------------------

# named-bits: [('bikeRevocableLane', '0'), ('pedestrianUseAllowed', '1'), ('isBikeFlyOverLane', '2'), ('fixedCycleTime', '3'), ('biDirectionalCycleTimes', '4'), ('isolatedByBarrier', '5'), ('unsignalizedSegmentsPresent', '6')]
# size: [16]
string value
