# ------------------------------------------------------------------------------
# NodeAttributeXY ::= ENUMERATED {
#   reserved,  
#   stopLine,
#   roundedCapStyleA,
#   roundedCapStyleB,
#   mergePoint,
#   divergePoint,
#   downstreamStopLine,
#   downstreamStartNode,
#   closedToTraffic,
#   safeIsland,
#   curbPresentAtStepOff,
#   hydrantPresent,
#   ...
# }
# ------------------------------------------------------------------------------

uint8 RESERVED = 0
uint8 STOP_LINE = 1
uint8 ROUNDED_CAP_STYLE_A = 2
uint8 ROUNDED_CAP_STYLE_B = 3
uint8 MERGE_POINT = 4
uint8 DIVERGE_POINT = 5
uint8 DOWNSTREAM_STOP_LINE = 6
uint8 DOWNSTREAM_START_NODE = 7
uint8 CLOSED_TO_TRAFFIC = 8
uint8 SAFE_ISLAND = 9
uint8 CURB_PRESENT_AT_STEP_OFF = 10
uint8 HYDRANT_PRESENT = 11
uint8 value
