# ------------------------------------------------------------------------------
# PtvRequestType  ::= ENUMERATED {
#   preRequest,
#   mainRequest,
#   doorCloseRequest,
#   cancelRequest,
#   emergencyRequest,
#   ...
# }
# ------------------------------------------------------------------------------

uint8 PRE_REQUEST = 0
uint8 MAIN_REQUEST = 1
uint8 DOOR_CLOSE_REQUEST = 2
uint8 CANCEL_REQUEST = 3
uint8 EMERGENCY_REQUEST = 4
uint8 value
