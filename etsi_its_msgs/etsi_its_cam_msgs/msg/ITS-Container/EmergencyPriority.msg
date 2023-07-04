# ------------------------------------------------------------------------------
# EmergencyPriority ::= BIT STRING {requestForRightOfWay(0), requestForFreeCrossingAtATrafficLight(1)} (SIZE(2))
# ------------------------------------------------------------------------------

bool[2] value
uint8 INDEX_REQUEST_FOR_RIGHT_OF_WAY = 0
uint8 INDEX_REQUEST_FOR_FREE_CROSSING_AT_A_TRAFFIC_LIGHT = 1

