# ------------------------------------------------------------------------------
# RejectedReason ::= ENUMERATED {
#   unknown,
#   exceptionalCondition,
#   maxWaitingTimeExceeded,
#   ptPriorityDisabled,
#   higherPTPriorityGranted,
#   vehicleTrackingUnknown,
#   ...
# }
# ------------------------------------------------------------------------------

uint8 UNKNOWN = 0
uint8 EXCEPTIONAL_CONDITION = 1
uint8 MAX_WAITING_TIME_EXCEEDED = 2
uint8 PT_PRIORITY_DISABLED = 3
uint8 HIGHER_P_T_PRIORITY_GRANTED = 4
uint8 VEHICLE_TRACKING_UNKNOWN = 5
uint8 value
