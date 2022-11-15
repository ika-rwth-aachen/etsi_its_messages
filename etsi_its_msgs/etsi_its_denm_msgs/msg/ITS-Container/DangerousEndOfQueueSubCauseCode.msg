# ------------------------------------------------------------------------------
# DangerousEndOfQueueSubCauseCode ::= INTEGER {unavailable(0), suddenEndOfQueue(1), queueOverHill(2), queueAroundBend(3), queueInTunnel(4)} (0..255)
# ------------------------------------------------------------------------------

# restricted-to: [(0, 255)]
uint8 UNAVAILABLE = 0
uint8 SUDDEN_END_OF_QUEUE = 1
uint8 QUEUE_OVER_HILL = 2
uint8 QUEUE_AROUND_BEND = 3
uint8 QUEUE_IN_TUNNEL = 4
uint8 value
