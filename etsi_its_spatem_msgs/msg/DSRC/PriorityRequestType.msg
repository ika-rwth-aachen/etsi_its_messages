# ------------------------------------------------------------------------------
# PriorityRequestType ::= ENUMERATED {
#   priorityRequestTypeReserved (0),  
#   priorityRequest             (1),   
#   priorityRequestUpdate       (2), 
#   priorityCancellation        (3),  
#   ...  
# }
# ------------------------------------------------------------------------------

uint8 PRIORITY_REQUEST_TYPE_RESERVED = 0
uint8 PRIORITY_REQUEST = 1
uint8 PRIORITY_REQUEST_UPDATE = 2
uint8 PRIORITY_CANCELLATION = 3
uint8 value
