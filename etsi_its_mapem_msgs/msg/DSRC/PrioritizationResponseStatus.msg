# ------------------------------------------------------------------------------
# PrioritizationResponseStatus ::= ENUMERATED {
#   unknown           (0),
#   requested         (1),
#   processing        (2),
#   watchOtherTraffic (3),
#   granted           (4),
#   rejected          (5),
#   maxPresence       (6),
#   reserviceLocked   (7),
#   ...
# }
# ------------------------------------------------------------------------------

uint8 UNKNOWN = 0
uint8 REQUESTED = 1
uint8 PROCESSING = 2
uint8 WATCH_OTHER_TRAFFIC = 3
uint8 GRANTED = 4
uint8 REJECTED = 5
uint8 MAX_PRESENCE = 6
uint8 RESERVICE_LOCKED = 7
uint8 value
